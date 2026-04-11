use crate::{
    dynamics::{
        joints::JointAxisMotion,
        solver::{
            solver_body::{SolverBody, SolverBodyInertia},
            xpbd::*,
        },
    },
    prelude::*,
};
use bevy::prelude::*;

/// Constraint data required by the XPBD constraint solver for a [`SixDofJoint`].
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, PartialEq)]
pub struct SixDofJointSolverData {
    /// Anchor offsets from centers of mass in world space.
    pub(super) world_r1: Vector,
    pub(super) world_r2: Vector,
    pub(super) center_difference: Vector,

    /// World-space joint basis rotations at the beginning of the step.
    pub(super) basis1_world: Quaternion,
    pub(super) basis2_world: Quaternion,

    /// Per-axis accumulated XPBD Lagrange multipliers.
    pub(super) linear_lagrange: [Scalar; 3],
    pub(super) angular_lagrange: [Scalar; 3],

    /// Total impulses for force/torque readback.
    pub(super) total_linear_impulse: Vector,
    pub(super) total_angular_impulse: Vector,
}

impl XpbdConstraintSolverData for SixDofJointSolverData {
    fn clear_lagrange_multipliers(&mut self) {
        self.linear_lagrange = [0.0; 3];
        self.angular_lagrange = [0.0; 3];
        self.total_linear_impulse = Vector::ZERO;
        self.total_angular_impulse = Vector::ZERO;
    }

    fn total_position_lagrange(&self) -> Vector {
        self.total_linear_impulse
    }

    fn total_rotation_lagrange(&self) -> AngularVector {
        self.total_angular_impulse
    }
}

impl XpbdConstraint<2> for SixDofJoint {
    type SolverData = SixDofJointSolverData;

    fn prepare(
        &mut self,
        bodies: [&RigidBodyQueryReadOnlyItem; 2],
        solver_data: &mut SixDofJointSolverData,
    ) {
        let [body1, body2] = bodies;

        let Some(local_anchor1) = self.local_anchor1() else {
            return;
        };
        let Some(local_anchor2) = self.local_anchor2() else {
            return;
        };
        let Some(local_basis1) = self.local_basis1() else {
            return;
        };
        let Some(local_basis2) = self.local_basis2() else {
            return;
        };

        let rot1_mat = Matrix::from_quat(body1.rotation.0);
        let rot2_mat = Matrix::from_quat(body2.rotation.0);

        solver_data.world_r1 = rot1_mat * (local_anchor1 - body1.center_of_mass.0);
        solver_data.world_r2 = rot2_mat * (local_anchor2 - body2.center_of_mass.0);
        solver_data.center_difference = (body2.position.0 - body1.position.0)
            + (body2.rotation * body2.center_of_mass.0 - body1.rotation * body1.center_of_mass.0);

        solver_data.basis1_world = body1.rotation.0 * local_basis1;
        solver_data.basis2_world = body2.rotation.0 * local_basis2;
    }

    fn solve(
        &mut self,
        bodies: [&mut SolverBody; 2],
        inertias: [&SolverBodyInertia; 2],
        solver_data: &mut SixDofJointSolverData,
        dt: Scalar,
    ) {
        let [body1, body2] = bodies;
        let [inertia1, inertia2] = inertias;

        // Angular constraints first (twist, swing1, swing2).
        // Each recomputes the relative rotation for accuracy since axes are coupled.
        for i in 0..3 {
            if self.angular_motion[i] == JointAxisMotion::Free {
                continue;
            }
            solve_angular_dof(
                self,
                body1,
                body2,
                inertia1,
                inertia2,
                solver_data,
                i,
                dt,
            );
        }

        // Then linear constraints (X, Y, Z).
        for i in 0..3 {
            if self.linear_motion[i] == JointAxisMotion::Free {
                continue;
            }
            solve_linear_dof(
                self,
                body1,
                body2,
                inertia1,
                inertia2,
                solver_data,
                i,
                dt,
            );
        }
    }
}

// --- Solver helpers (free functions to avoid borrow issues with &mut self) ---

fn current_basis1(body1: &SolverBody, solver_data: &SixDofJointSolverData) -> Quaternion {
    body1.delta_rotation.0 * solver_data.basis1_world
}

fn current_basis2(body2: &SolverBody, solver_data: &SixDofJointSolverData) -> Quaternion {
    body2.delta_rotation.0 * solver_data.basis2_world
}

fn basis_axis(basis: Quaternion, index: usize) -> Vector {
    match index {
        0 => basis * Vector::X,
        1 => basis * Vector::Y,
        _ => basis * Vector::Z,
    }
}

/// Wrap angle to `[-PI, PI]`.
fn wrap_angle(mut angle: Scalar) -> Scalar {
    while angle > PI {
        angle -= TAU;
    }
    while angle < -PI {
        angle += TAU;
    }
    angle
}

/// Swing-twist decomposition about the X axis.
///
/// Returns `(swing_quaternion, twist_quaternion)`.
fn decompose_swing_twist_x(q: Quaternion) -> (Quaternion, Quaternion) {
    let twist = Quaternion::from_xyzw(q.x, 0.0, 0.0, q.w);
    let twist = if twist.length_squared() > Scalar::EPSILON {
        twist.normalize()
    } else {
        Quaternion::IDENTITY
    };
    let swing = q * twist.inverse();
    (swing, twist)
}

/// Extracts scalar twist/swing1/swing2 angles from a relative quaternion.
///
/// Convention: twist = rotation about X, swing1 = about Y, swing2 = about Z.
fn extract_angular_coordinates(q_rel: Quaternion) -> [Scalar; 3] {
    let (swing, twist) = decompose_swing_twist_x(q_rel);

    let twist_angle = wrap_angle(2.0 * twist.x.atan2(twist.w));

    // Swing quaternion rotates X to some direction; measure deflection into Y and Z.
    let swung_x = swing * Vector::X;
    let swing1 = wrap_angle(swung_x.z.atan2(swung_x.x));
    let swing2 = wrap_angle((-swung_x.y).atan2(swung_x.x));

    [twist_angle, swing1, swing2]
}

/// Returns `Some(error)` if `value` is outside `[min, max]`, else `None`.
fn scalar_limit_error(value: Scalar, min: Scalar, max: Scalar) -> Option<Scalar> {
    if value < min {
        Some(value - min)
    } else if value > max {
        Some(value - max)
    } else {
        None
    }
}

#[allow(clippy::too_many_arguments)]
fn solve_angular_dof(
    joint: &SixDofJoint,
    body1: &mut SolverBody,
    body2: &mut SolverBody,
    inertia1: &SolverBodyInertia,
    inertia2: &SolverBodyInertia,
    solver_data: &mut SixDofJointSolverData,
    axis_index: usize,
    dt: Scalar,
) {
    let basis1 = current_basis1(body1, solver_data);
    let basis2 = current_basis2(body2, solver_data);
    let q_rel = basis1.inverse() * basis2;
    let angles = extract_angular_coordinates(q_rel);
    let angle = angles[axis_index];

    let c = match joint.angular_motion[axis_index] {
        JointAxisMotion::Free => return,
        JointAxisMotion::Locked => angle,
        JointAxisMotion::Limited => {
            let limit = joint.angular_limits[axis_index];
            match scalar_limit_error(angle, limit.min, limit.max) {
                Some(err) => err,
                None => return,
            }
        }
    };

    if c.abs() <= Scalar::EPSILON {
        return;
    }

    let axis_world = basis_axis(basis1, axis_index);
    let inv_i1 = inertia1.effective_inv_angular_inertia();
    let inv_i2 = inertia2.effective_inv_angular_inertia();

    let w1 = axis_world.dot(inv_i1 * axis_world);
    let w2 = axis_world.dot(inv_i2 * axis_world);

    let lambda = &mut solver_data.angular_lagrange[axis_index];
    let dl = compute_lagrange_update(*lambda, c, &[w1, w2], joint.angular_compliance[axis_index], dt);
    *lambda += dl;

    // For angular constraints on q_rel = basis1⁻¹ * basis2, the sign convention
    // is opposite to the positional case: impulse = dl * axis (no negation).
    let impulse = dl * axis_world;
    solver_data.total_angular_impulse += impulse;

    let delta_quat1 = Quaternion::from_scaled_axis(inv_i1 * impulse);
    body1.delta_rotation.0 = delta_quat1 * body1.delta_rotation.0;

    let delta_quat2 = Quaternion::from_scaled_axis(inv_i2 * (-impulse));
    body2.delta_rotation.0 = delta_quat2 * body2.delta_rotation.0;
}

#[allow(clippy::too_many_arguments)]
fn solve_linear_dof(
    joint: &SixDofJoint,
    body1: &mut SolverBody,
    body2: &mut SolverBody,
    inertia1: &SolverBodyInertia,
    inertia2: &SolverBodyInertia,
    solver_data: &mut SixDofJointSolverData,
    axis_index: usize,
    dt: Scalar,
) {
    let basis1 = current_basis1(body1, solver_data);
    let axis_world = basis_axis(basis1, axis_index);

    let world_r1 = body1.delta_rotation * solver_data.world_r1;
    let world_r2 = body2.delta_rotation * solver_data.world_r2;
    let separation = (body2.delta_position - body1.delta_position)
        + (world_r2 - world_r1)
        + solver_data.center_difference;

    let value = separation.dot(axis_world);

    let c = match joint.linear_motion[axis_index] {
        JointAxisMotion::Free => return,
        JointAxisMotion::Locked => value,
        JointAxisMotion::Limited => {
            let limit = joint.linear_limits[axis_index];
            match scalar_limit_error(value, limit.min, limit.max) {
                Some(err) => err,
                None => return,
            }
        }
    };

    if c.abs() <= Scalar::EPSILON {
        return;
    }

    let inv_mass1 = inertia1.effective_inv_mass();
    let inv_mass2 = inertia2.effective_inv_mass();
    let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
    let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();

    // Generalized inverse mass = linear + angular contribution via lever arm.
    let r1_cross_n = world_r1.cross(axis_world);
    let r2_cross_n = world_r2.cross(axis_world);
    let w1 = inv_mass1.max_element() + r1_cross_n.dot(inv_angular_inertia1 * r1_cross_n);
    let w2 = inv_mass2.max_element() + r2_cross_n.dot(inv_angular_inertia2 * r2_cross_n);

    let lambda = &mut solver_data.linear_lagrange[axis_index];
    let dl = compute_lagrange_update(*lambda, c, &[w1, w2], joint.linear_compliance[axis_index], dt);
    *lambda += dl;

    // Negate: XPBD gradient for body1 is -axis, so applied impulse is -dl * axis.
    let impulse = -dl * axis_world;
    solver_data.total_linear_impulse += impulse;

    // Apply linear correction.
    body1.delta_position += impulse * inv_mass1;
    body2.delta_position -= impulse * inv_mass2;

    // Apply angular correction from lever arm.
    let delta_quat1 = Quaternion::from_scaled_axis(inv_angular_inertia1 * world_r1.cross(impulse));
    body1.delta_rotation.0 = delta_quat1 * body1.delta_rotation.0;

    let delta_quat2 = Quaternion::from_scaled_axis(inv_angular_inertia2 * world_r2.cross(-impulse));
    body2.delta_rotation.0 = delta_quat2 * body2.delta_rotation.0;
}

impl PositionConstraint for SixDofJoint {}

impl AngularConstraint for SixDofJoint {}
