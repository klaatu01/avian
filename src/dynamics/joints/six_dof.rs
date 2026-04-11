use crate::{
    dynamics::joints::{EntityConstraint, JointSystems},
    prelude::*,
};
use bevy::{
    ecs::{
        entity::{EntityMapper, MapEntities},
        reflect::ReflectMapEntities,
    },
    prelude::*,
};

/// Defines the motion behavior of a single degree of freedom in a [`SixDofJoint`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub enum JointAxisMotion {
    /// The axis is free, allowing unrestricted relative motion.
    Free,
    /// The axis is locked, preventing any relative motion.
    #[default]
    Locked,
    /// The axis allows relative motion within configured limits.
    Limited,
}

/// The six configurable axes of a [`SixDofJoint`].
///
/// The axis directions are defined by the joint frame basis:
///
/// - Linear X / Y / Z are translations along the joint frame axes.
/// - Twist is rotation about the joint frame X axis.
/// - Swing1 is rotation about the joint frame Y axis.
/// - Swing2 is rotation about the joint frame Z axis.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub enum SixDofAxis {
    /// Translation along the joint frame's local X axis.
    LinearX,
    /// Translation along the joint frame's local Y axis.
    LinearY,
    /// Translation along the joint frame's local Z axis.
    LinearZ,
    /// Rotation about the joint frame's local X axis (twist).
    Twist,
    /// Rotation about the joint frame's local Y axis (swing 1).
    Swing1,
    /// Rotation about the joint frame's local Z axis (swing 2).
    Swing2,
}

/// A generic PhysX/Bullet-style 6 degrees-of-freedom [joint](dynamics::joints).
///
/// This joint exposes six independently configurable scalar DOFs:
///
/// - Linear X, Y, Z
/// - Twist   (rotation about local joint X)
/// - Swing1  (rotation about local joint Y)
/// - Swing2  (rotation about local joint Z)
///
/// The axes are defined by the [`JointFrame`] basis on each body.
///
/// By default all six DOFs are locked, making the joint behave like a [`FixedJoint`].
#[derive(Component, Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, MapEntities, PartialEq)]
pub struct SixDofJoint {
    /// The first body constrained by the joint.
    pub body1: Entity,
    /// The second body constrained by the joint.
    pub body2: Entity,
    /// The reference frame of the first body.
    pub frame1: JointFrame,
    /// The reference frame of the second body.
    pub frame2: JointFrame,

    /// Motion for linear X, Y, Z.
    pub linear_motion: [JointAxisMotion; 3],
    /// Motion for twist, swing1, swing2.
    pub angular_motion: [JointAxisMotion; 3],

    /// Limits for linear X, Y, Z.
    pub linear_limits: [DistanceLimit; 3],
    /// Limits for twist, swing1, swing2.
    pub angular_limits: [AngleLimit; 3],

    /// Compliance for linear X, Y, Z.
    pub linear_compliance: [Scalar; 3],
    /// Compliance for twist, swing1, swing2.
    pub angular_compliance: [Scalar; 3],
}

impl EntityConstraint<2> for SixDofJoint {
    fn entities(&self) -> [Entity; 2] {
        [self.body1, self.body2]
    }
}

impl SixDofJoint {
    /// Index of the linear X axis in the motion/limit/compliance arrays.
    pub const LINEAR_X: usize = 0;
    /// Index of the linear Y axis in the motion/limit/compliance arrays.
    pub const LINEAR_Y: usize = 1;
    /// Index of the linear Z axis in the motion/limit/compliance arrays.
    pub const LINEAR_Z: usize = 2;

    /// Index of the twist axis in the angular motion/limit/compliance arrays.
    pub const TWIST: usize = 0;
    /// Index of the swing1 axis in the angular motion/limit/compliance arrays.
    pub const SWING1: usize = 1;
    /// Index of the swing2 axis in the angular motion/limit/compliance arrays.
    pub const SWING2: usize = 2;

    /// Creates a new [`SixDofJoint`] between two entities.
    ///
    /// By default all six DOFs are locked.
    #[inline]
    pub const fn new(body1: Entity, body2: Entity) -> Self {
        Self {
            body1,
            body2,
            frame1: JointFrame::IDENTITY,
            frame2: JointFrame::IDENTITY,
            linear_motion: [JointAxisMotion::Locked; 3],
            angular_motion: [JointAxisMotion::Locked; 3],
            linear_limits: [DistanceLimit::ZERO; 3],
            angular_limits: [AngleLimit::ZERO; 3],
            linear_compliance: [0.0; 3],
            angular_compliance: [0.0; 3],
        }
    }

    #[inline]
    fn linear_index(axis: SixDofAxis) -> Option<usize> {
        match axis {
            SixDofAxis::LinearX => Some(Self::LINEAR_X),
            SixDofAxis::LinearY => Some(Self::LINEAR_Y),
            SixDofAxis::LinearZ => Some(Self::LINEAR_Z),
            _ => None,
        }
    }

    #[inline]
    fn angular_index(axis: SixDofAxis) -> Option<usize> {
        match axis {
            SixDofAxis::Twist => Some(Self::TWIST),
            SixDofAxis::Swing1 => Some(Self::SWING1),
            SixDofAxis::Swing2 => Some(Self::SWING2),
            _ => None,
        }
    }

    // --- Frame setters ---

    /// Sets the local [`JointFrame`] of the first body.
    #[inline]
    pub fn with_local_frame1(mut self, frame: impl Into<Isometry>) -> Self {
        self.frame1 = JointFrame::local(frame);
        self
    }

    /// Sets the local [`JointFrame`] of the second body.
    #[inline]
    pub fn with_local_frame2(mut self, frame: impl Into<Isometry>) -> Self {
        self.frame2 = JointFrame::local(frame);
        self
    }

    /// Sets the global anchor point on both bodies.
    #[inline]
    pub const fn with_anchor(mut self, anchor: Vector) -> Self {
        self.frame1.anchor = JointAnchor::FromGlobal(anchor);
        self.frame2.anchor = JointAnchor::FromGlobal(anchor);
        self
    }

    /// Sets the local anchor point on the first body.
    #[inline]
    pub const fn with_local_anchor1(mut self, anchor: Vector) -> Self {
        self.frame1.anchor = JointAnchor::Local(anchor);
        self
    }

    /// Sets the local anchor point on the second body.
    #[inline]
    pub const fn with_local_anchor2(mut self, anchor: Vector) -> Self {
        self.frame2.anchor = JointAnchor::Local(anchor);
        self
    }

    /// Sets the global basis for both bodies.
    #[inline]
    pub fn with_basis(mut self, basis: impl Into<Rot>) -> Self {
        let basis = basis.into();
        self.frame1.basis = JointBasis::FromGlobal(basis);
        self.frame2.basis = JointBasis::FromGlobal(basis);
        self
    }

    /// Sets the local basis for the first body.
    #[inline]
    pub fn with_local_basis1(mut self, basis: impl Into<Rot>) -> Self {
        self.frame1.basis = JointBasis::Local(basis.into());
        self
    }

    /// Sets the local basis for the second body.
    #[inline]
    pub fn with_local_basis2(mut self, basis: impl Into<Rot>) -> Self {
        self.frame2.basis = JointBasis::Local(basis.into());
        self
    }

    // --- Frame accessors ---

    /// Returns the local [`JointFrame`] of the first body.
    #[inline]
    pub fn local_frame1(&self) -> Option<Isometry> {
        self.frame1.get_local_isometry()
    }

    /// Returns the local [`JointFrame`] of the second body.
    #[inline]
    pub fn local_frame2(&self) -> Option<Isometry> {
        self.frame2.get_local_isometry()
    }

    /// Returns the local anchor point on the first body.
    #[inline]
    pub const fn local_anchor1(&self) -> Option<Vector> {
        match self.frame1.anchor {
            JointAnchor::Local(anchor) => Some(anchor),
            _ => None,
        }
    }

    /// Returns the local anchor point on the second body.
    #[inline]
    pub const fn local_anchor2(&self) -> Option<Vector> {
        match self.frame2.anchor {
            JointAnchor::Local(anchor) => Some(anchor),
            _ => None,
        }
    }

    /// Returns the local basis of the first body.
    #[inline]
    pub const fn local_basis1(&self) -> Option<Rot> {
        match self.frame1.basis {
            JointBasis::Local(basis) => Some(basis),
            _ => None,
        }
    }

    /// Returns the local basis of the second body.
    #[inline]
    pub const fn local_basis2(&self) -> Option<Rot> {
        match self.frame2.basis {
            JointBasis::Local(basis) => Some(basis),
            _ => None,
        }
    }

    // --- Motion setters ---

    /// Sets motion for any of the six DOFs.
    #[inline]
    pub fn with_motion(mut self, axis: SixDofAxis, motion: JointAxisMotion) -> Self {
        if let Some(i) = Self::linear_index(axis) {
            self.linear_motion[i] = motion;
        } else if let Some(i) = Self::angular_index(axis) {
            self.angular_motion[i] = motion;
        }
        self
    }

    /// Sets the motion mode for linear X.
    #[inline]
    pub const fn with_linear_x(mut self, motion: JointAxisMotion) -> Self {
        self.linear_motion[Self::LINEAR_X] = motion;
        self
    }

    /// Sets the motion mode for linear Y.
    #[inline]
    pub const fn with_linear_y(mut self, motion: JointAxisMotion) -> Self {
        self.linear_motion[Self::LINEAR_Y] = motion;
        self
    }

    /// Sets the motion mode for linear Z.
    #[inline]
    pub const fn with_linear_z(mut self, motion: JointAxisMotion) -> Self {
        self.linear_motion[Self::LINEAR_Z] = motion;
        self
    }

    /// Sets the motion mode for all three linear axes.
    #[inline]
    pub const fn with_all_linear(mut self, motion: JointAxisMotion) -> Self {
        self.linear_motion = [motion; 3];
        self
    }

    /// Sets the motion mode for twist (rotation about local X).
    #[inline]
    pub const fn with_twist(mut self, motion: JointAxisMotion) -> Self {
        self.angular_motion[Self::TWIST] = motion;
        self
    }

    /// Sets the motion mode for swing1 (rotation about local Y).
    #[inline]
    pub const fn with_swing1(mut self, motion: JointAxisMotion) -> Self {
        self.angular_motion[Self::SWING1] = motion;
        self
    }

    /// Sets the motion mode for swing2 (rotation about local Z).
    #[inline]
    pub const fn with_swing2(mut self, motion: JointAxisMotion) -> Self {
        self.angular_motion[Self::SWING2] = motion;
        self
    }

    /// Sets the motion mode for all three angular axes.
    #[inline]
    pub const fn with_all_angular(mut self, motion: JointAxisMotion) -> Self {
        self.angular_motion = [motion; 3];
        self
    }

    // --- Limit setters ---

    /// Sets limits for any single DOF and marks it as limited.
    #[inline]
    pub fn with_limit(mut self, axis: SixDofAxis, min: Scalar, max: Scalar) -> Self {
        if let Some(i) = Self::linear_index(axis) {
            self.linear_limits[i] = DistanceLimit::new(min, max);
            self.linear_motion[i] = JointAxisMotion::Limited;
        } else if let Some(i) = Self::angular_index(axis) {
            self.angular_limits[i] = AngleLimit::new(min, max);
            self.angular_motion[i] = JointAxisMotion::Limited;
        }
        self
    }

    /// Sets limits for all three linear axes and marks them as limited.
    #[inline]
    pub fn with_linear_limits(mut self, lower: Vector, upper: Vector) -> Self {
        self.linear_limits[Self::LINEAR_X] = DistanceLimit::new(lower.x, upper.x);
        self.linear_limits[Self::LINEAR_Y] = DistanceLimit::new(lower.y, upper.y);
        self.linear_limits[Self::LINEAR_Z] = DistanceLimit::new(lower.z, upper.z);
        self.linear_motion = [JointAxisMotion::Limited; 3];
        self
    }

    /// Sets limits for all three angular axes and marks them as limited.
    #[inline]
    pub fn with_angular_limits(mut self, lower: Vector, upper: Vector) -> Self {
        self.angular_limits[Self::TWIST] = AngleLimit::new(lower.x, upper.x);
        self.angular_limits[Self::SWING1] = AngleLimit::new(lower.y, upper.y);
        self.angular_limits[Self::SWING2] = AngleLimit::new(lower.z, upper.z);
        self.angular_motion = [JointAxisMotion::Limited; 3];
        self
    }

    /// Sets limits for linear X and marks it as limited.
    #[inline]
    pub fn with_linear_x_limits(mut self, min: Scalar, max: Scalar) -> Self {
        self.linear_limits[Self::LINEAR_X] = DistanceLimit::new(min, max);
        self.linear_motion[Self::LINEAR_X] = JointAxisMotion::Limited;
        self
    }

    /// Sets limits for linear Y and marks it as limited.
    #[inline]
    pub fn with_linear_y_limits(mut self, min: Scalar, max: Scalar) -> Self {
        self.linear_limits[Self::LINEAR_Y] = DistanceLimit::new(min, max);
        self.linear_motion[Self::LINEAR_Y] = JointAxisMotion::Limited;
        self
    }

    /// Sets limits for linear Z and marks it as limited.
    #[inline]
    pub fn with_linear_z_limits(mut self, min: Scalar, max: Scalar) -> Self {
        self.linear_limits[Self::LINEAR_Z] = DistanceLimit::new(min, max);
        self.linear_motion[Self::LINEAR_Z] = JointAxisMotion::Limited;
        self
    }

    /// Sets limits for twist and marks it as limited.
    #[inline]
    pub fn with_twist_limits(mut self, min: Scalar, max: Scalar) -> Self {
        self.angular_limits[Self::TWIST] = AngleLimit::new(min, max);
        self.angular_motion[Self::TWIST] = JointAxisMotion::Limited;
        self
    }

    /// Sets limits for swing1 and marks it as limited.
    #[inline]
    pub fn with_swing1_limits(mut self, min: Scalar, max: Scalar) -> Self {
        self.angular_limits[Self::SWING1] = AngleLimit::new(min, max);
        self.angular_motion[Self::SWING1] = JointAxisMotion::Limited;
        self
    }

    /// Sets limits for swing2 and marks it as limited.
    #[inline]
    pub fn with_swing2_limits(mut self, min: Scalar, max: Scalar) -> Self {
        self.angular_limits[Self::SWING2] = AngleLimit::new(min, max);
        self.angular_motion[Self::SWING2] = JointAxisMotion::Limited;
        self
    }

    // --- Compliance setters ---

    /// Sets compliance for any single DOF.
    #[inline]
    pub const fn with_compliance(mut self, axis: SixDofAxis, compliance: Scalar) -> Self {
        match axis {
            SixDofAxis::LinearX => self.linear_compliance[Self::LINEAR_X] = compliance,
            SixDofAxis::LinearY => self.linear_compliance[Self::LINEAR_Y] = compliance,
            SixDofAxis::LinearZ => self.linear_compliance[Self::LINEAR_Z] = compliance,
            SixDofAxis::Twist => self.angular_compliance[Self::TWIST] = compliance,
            SixDofAxis::Swing1 => self.angular_compliance[Self::SWING1] = compliance,
            SixDofAxis::Swing2 => self.angular_compliance[Self::SWING2] = compliance,
        }
        self
    }

    /// Sets compliance for all three linear axes.
    #[inline]
    pub const fn with_all_linear_compliance(mut self, compliance: Scalar) -> Self {
        self.linear_compliance = [compliance; 3];
        self
    }

    /// Sets compliance for all three angular axes.
    #[inline]
    pub const fn with_all_angular_compliance(mut self, compliance: Scalar) -> Self {
        self.angular_compliance = [compliance; 3];
        self
    }
}

impl MapEntities for SixDofJoint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.body1 = entity_mapper.get_mapped(self.body1);
        self.body2 = entity_mapper.get_mapped(self.body2);
    }
}

pub(super) fn plugin(app: &mut App) {
    app.add_systems(
        PhysicsSchedule,
        update_local_frames.in_set(JointSystems::PrepareLocalFrames),
    );
}

fn update_local_frames(
    mut joints: Query<&mut SixDofJoint, Changed<SixDofJoint>>,
    bodies: Query<(&Position, &Rotation)>,
) {
    for mut joint in &mut joints {
        if matches!(joint.frame1.anchor, JointAnchor::Local(_))
            && matches!(joint.frame2.anchor, JointAnchor::Local(_))
            && matches!(joint.frame1.basis, JointBasis::Local(_))
            && matches!(joint.frame2.basis, JointBasis::Local(_))
        {
            continue;
        }

        let Ok([(pos1, rot1), (pos2, rot2)]) = bodies.get_many(joint.entities()) else {
            continue;
        };

        let [frame1, frame2] =
            JointFrame::compute_local(joint.frame1, joint.frame2, pos1.0, pos2.0, rot1, rot2);
        joint.frame1 = frame1;
        joint.frame2 = frame2;
    }
}

#[cfg(feature = "debug-plugin")]
impl DebugRenderConstraint<2> for SixDofJoint {
    type Context = ();

    fn debug_render(
        &self,
        positions: [Vector; 2],
        rotations: [Rotation; 2],
        _context: &mut Self::Context,
        gizmos: &mut Gizmos<PhysicsGizmos>,
        config: &PhysicsGizmos,
    ) {
        let [pos1, pos2] = positions;
        let [rot1, rot2] = rotations;

        let Some(local_anchor1) = self.local_anchor1() else {
            return;
        };
        let Some(local_anchor2) = self.local_anchor2() else {
            return;
        };

        let anchor1 = pos1 + rot1 * local_anchor1;
        let anchor2 = pos2 + rot2 * local_anchor2;

        if let Some(anchor_color) = config.joint_anchor_color {
            gizmos.draw_line(pos1, anchor1, anchor_color);
            gizmos.draw_line(pos2, anchor2, anchor_color);
        }

        if let Some(color) = config.joint_separation_color {
            gizmos.draw_line(anchor1, anchor2, color);
        }
    }
}
