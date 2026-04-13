use crate::{
    dynamics::{
        joints::{EntityConstraint, JointAxisMotion},
        solver::{
            solver_body::{SolverBody, SolverBodyInertia},
            xpbd::*,
        },
    },
    prelude::*,
};
use bevy::prelude::*;

/// Constraint data required by the XPBD constraint solver for a [`SixDofJoint`].
///
/// Lock constraints use a single bilateral lambda per axis.
/// Limit constraints use two unilateral lambdas (lower + upper) per axis,
/// each clamped to non-negative values.
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

    // --- Per-axis Lagrange multipliers ---
    // Locks: one bilateral (unrestricted sign) lambda per axis.
    // Limits: two unilateral (>= 0) lambdas per axis (lower, upper).

    /// Bilateral lambda for locked linear axes `[X, Y, Z]`.
    pub(super) linear_lock_lambda: [Scalar; 3],
    /// Unilateral lambda for lower linear limits `[X, Y, Z]`.
    pub(super) linear_lower_lambda: [Scalar; 3],
    /// Unilateral lambda for upper linear limits `[X, Y, Z]`.
    pub(super) linear_upper_lambda: [Scalar; 3],

    /// Bilateral lambda for locked angular axes `[Twist, Swing1, Swing2]`.
    pub(super) angular_lock_lambda: [Scalar; 3],
    /// Unilateral lambda for lower angular limits `[Twist, Swing1, Swing2]`.
    pub(super) angular_lower_lambda: [Scalar; 3],
    /// Unilateral lambda for upper angular limits `[Twist, Swing1, Swing2]`.
    pub(super) angular_upper_lambda: [Scalar; 3],

    /// Per-axis angular motor Lagrange multipliers `[Twist, Swing1, Swing2]`.
    pub(super) angular_motor_lagrange: [Scalar; 3],

    /// Total impulses for force/torque readback.
    pub(super) total_linear_impulse: Vector,
    pub(super) total_angular_impulse: Vector,

    /// Pre-solve `delta_rotation` of body1/body2, snapshotted at the start
    /// of every substep (before any joint solves run). The motor solver
    /// uses these to derive a "live" intra-substep angular velocity
    ///
    ///     live_ω = body.angular_velocity + 2 * (delta_rot * pre⁻¹).xyz / dt
    ///
    /// so that when adjacent motorized joints share a body, the second
    /// joint's motor sees ω that reflects the first joint's in-substep
    /// corrections, instead of a stale start-of-substep snapshot.
    pub(super) pre_solve_delta_rotation_1: Quaternion,
    pub(super) pre_solve_delta_rotation_2: Quaternion,
}

impl XpbdConstraintSolverData for SixDofJointSolverData {
    fn clear_lagrange_multipliers(&mut self) {
        self.linear_lock_lambda = [0.0; 3];
        self.linear_lower_lambda = [0.0; 3];
        self.linear_upper_lambda = [0.0; 3];
        self.angular_lock_lambda = [0.0; 3];
        self.angular_lower_lambda = [0.0; 3];
        self.angular_upper_lambda = [0.0; 3];
        self.angular_motor_lagrange = [0.0; 3];
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
        // When all 3 are locked, use a combined solve (same as FixedAngleConstraintShared)
        // to avoid per-axis cross-coupling that causes chain instability.
        let all_angular_locked = self.angular_motion[0] == JointAxisMotion::Locked
            && self.angular_motion[1] == JointAxisMotion::Locked
            && self.angular_motion[2] == JointAxisMotion::Locked;

        if all_angular_locked {
            solve_angular_combined(body1, body2, inertia1, inertia2, solver_data, dt);
        } else {
            // Mixed case: solve locked DOFs via the stable quaternion-error path first,
            // then limited DOFs via per-axis decomposition.
            let has_any_locked = self.angular_motion.iter().any(|m| *m == JointAxisMotion::Locked);
            if has_any_locked {
                solve_angular_locked_axes(self, body1, body2, inertia1, inertia2, solver_data, dt);
            }
            for i in 0..3 {
                if self.angular_motion[i] != JointAxisMotion::Limited {
                    continue;
                }
                solve_angular_dof(self, body1, body2, inertia1, inertia2, solver_data, i, dt);
            }
        }

        // Angular motors (after constraints so limits take priority).
        for i in 0..3 {
            if !self.angular_motors[i].enabled {
                continue;
            }
            solve_angular_motor(self, body1, body2, inertia1, inertia2, solver_data, i, dt);
        }

        // Linear constraints.
        let all_linear_locked = self.linear_motion[0] == JointAxisMotion::Locked
            && self.linear_motion[1] == JointAxisMotion::Locked
            && self.linear_motion[2] == JointAxisMotion::Locked;

        if all_linear_locked {
            solve_linear_combined(body1, body2, inertia1, inertia2, solver_data, dt);
        } else {
            for i in 0..3 {
                if self.linear_motion[i] == JointAxisMotion::Free {
                    continue;
                }
                solve_linear_dof(self, body1, body2, inertia1, inertia2, solver_data, i, dt);
            }
        }

    }
}

// --- Helpers ---

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


// --- Combined angular solver (all 3 locked) ---

/// Solves all 3 locked angular axes as a single combined correction.
/// Uses the same -2*q.xyz() approach as FixedAngleConstraintShared.
fn solve_angular_combined(
    body1: &mut SolverBody,
    body2: &mut SolverBody,
    inertia1: &SolverBodyInertia,
    inertia2: &SolverBodyInertia,
    solver_data: &mut SixDofJointSolverData,
    dt: Scalar,
) {
    let basis1 = current_basis1(body1, solver_data);
    let basis2 = current_basis2(body2, solver_data);
    let q_error = basis1 * basis2.inverse();
    let error = -2.0 * Vector::new(q_error.x, q_error.y, q_error.z);

    let angle = error.length();
    if angle <= Scalar::EPSILON {
        return;
    }

    let axis = error / angle;
    let inv_i1 = inertia1.effective_inv_angular_inertia();
    let inv_i2 = inertia2.effective_inv_angular_inertia();
    let w1 = axis.dot(inv_i1 * axis);
    let w2 = axis.dot(inv_i2 * axis);

    let dl = compute_lagrange_update(0.0, angle, &[w1, w2], 0.0, dt);
    let impulse = -dl * axis;
    solver_data.total_angular_impulse += impulse;

    let dq1 = Quaternion::from_scaled_axis(inv_i1 * impulse);
    body1.delta_rotation.0 = dq1 * body1.delta_rotation.0;
    let dq2 = Quaternion::from_scaled_axis(inv_i2 * (-impulse));
    body2.delta_rotation.0 = dq2 * body2.delta_rotation.0;
}

/// Solves only the LOCKED angular axes using the stable quaternion-error approach.
/// Each locked axis gets a scalar constraint from the projected error, applied individually
/// but using the singularity-free `-2*q.xyz()` error vector.
#[allow(clippy::too_many_arguments)]
fn solve_angular_locked_axes(
    joint: &SixDofJoint,
    body1: &mut SolverBody,
    body2: &mut SolverBody,
    inertia1: &SolverBodyInertia,
    inertia2: &SolverBodyInertia,
    solver_data: &mut SixDofJointSolverData,
    dt: Scalar,
) {
    let inv_i1 = inertia1.effective_inv_angular_inertia();
    let inv_i2 = inertia2.effective_inv_angular_inertia();

    for i in 0..3 {
        if joint.angular_motion[i] != JointAxisMotion::Locked {
            continue;
        }

        // Recompute error each axis (prior correction may have changed orientation).
        let basis1 = current_basis1(body1, solver_data);
        let basis2 = current_basis2(body2, solver_data);
        let q_error = basis1 * basis2.inverse();
        let error_world = -2.0 * Vector::new(q_error.x, q_error.y, q_error.z);
        let axis_world = basis_axis(basis1, i);
        let c = error_world.dot(axis_world);

        if c.abs() <= Scalar::EPSILON {
            continue;
        }

        let w1 = axis_world.dot(inv_i1 * axis_world);
        let w2 = axis_world.dot(inv_i2 * axis_world);
        let compliance = joint.angular_compliance[i];

        let lambda = &mut solver_data.angular_lock_lambda[i];
        let dl = compute_lagrange_update(*lambda, c, &[w1, w2], compliance, dt);
        *lambda += dl;

        apply_angular_impulse_d6(
            body1, body2, inv_i1, inv_i2,
            -dl * axis_world,
            &mut solver_data.total_angular_impulse,
        );
    }
}

// --- Per-axis angular DOF solver ---
//
// Uses the quaternion error `-2*q.xyz()` for ALL modes (locked and limited).
// This avoids the swing-twist decomposition which is unstable in chains
// due to `atan2` discontinuities and per-axis cross-coupling.
//
// For limited axes, the projected quaternion error is used as an approximation
// of the per-axis angle. This is accurate for small angles (typical limit violations)
// and degrades gracefully for large angles without singularities.

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
    let axis_world = basis_axis(basis1, axis_index);

    // Compute the per-axis error using the stable quaternion approach.
    let q_error = basis1 * basis2.inverse();
    let error_world = -2.0 * Vector::new(q_error.x, q_error.y, q_error.z);
    let angle = error_world.dot(axis_world);

    let inv_i1 = inertia1.effective_inv_angular_inertia();
    let inv_i2 = inertia2.effective_inv_angular_inertia();
    let w1 = axis_world.dot(inv_i1 * axis_world);
    let w2 = axis_world.dot(inv_i2 * axis_world);
    let compliance = joint.angular_compliance[axis_index];

    let c = match joint.angular_motion[axis_index] {
        JointAxisMotion::Free => return,
        JointAxisMotion::Locked => angle,
        JointAxisMotion::Limited => {
            let limit = joint.angular_limits[axis_index];
            if angle < limit.min {
                angle - limit.min
            } else if angle > limit.max {
                angle - limit.max
            } else {
                solver_data.angular_lock_lambda[axis_index] = 0.0;
                return;
            }
        }
    };

    if c.abs() <= Scalar::EPSILON {
        return;
    }

    let lambda = &mut solver_data.angular_lock_lambda[axis_index];
    let dl = compute_lagrange_update(*lambda, c, &[w1, w2], compliance, dt);
    *lambda += dl;

    // The `-2*q.xyz()` convention uses `-dl * axis` for the impulse
    // (same as FixedAngleConstraintShared's align_orientation).
    apply_angular_impulse_d6(
        body1, body2, inv_i1, inv_i2,
        -dl * axis_world,
        &mut solver_data.total_angular_impulse,
    );
}

// --- Combined linear solver (all 3 locked) ---

/// Solves all 3 locked linear axes as a single point constraint.
/// Uses the same approach as PointConstraintShared: one impulse along the
/// full separation direction. Much more stable for chains than per-axis.
fn solve_linear_combined(
    body1: &mut SolverBody,
    body2: &mut SolverBody,
    inertia1: &SolverBodyInertia,
    inertia2: &SolverBodyInertia,
    solver_data: &mut SixDofJointSolverData,
    dt: Scalar,
) {
    let world_r1 = body1.delta_rotation * solver_data.world_r1;
    let world_r2 = body2.delta_rotation * solver_data.world_r2;
    let separation = (body2.delta_position - body1.delta_position)
        + (world_r2 - world_r1)
        + solver_data.center_difference;

    let magnitude_sq = separation.length_squared();
    if magnitude_sq <= Scalar::EPSILON {
        return;
    }

    let magnitude = magnitude_sq.sqrt();
    let dir = -separation / magnitude;

    let inv_mass1 = inertia1.effective_inv_mass();
    let inv_mass2 = inertia2.effective_inv_mass();
    let inv_ai1 = inertia1.effective_inv_angular_inertia();
    let inv_ai2 = inertia2.effective_inv_angular_inertia();

    let r1_cross_n = world_r1.cross(dir);
    let r2_cross_n = world_r2.cross(dir);
    let w1 = inv_mass1.max_element() + r1_cross_n.dot(inv_ai1 * r1_cross_n);
    let w2 = inv_mass2.max_element() + r2_cross_n.dot(inv_ai2 * r2_cross_n);

    let dl = compute_lagrange_update(0.0, magnitude, &[w1, w2], 0.0, dt);
    let impulse = dl * dir;
    solver_data.total_linear_impulse += impulse;

    apply_positional_impulse_d6(
        body1, body2, inv_mass1, inv_mass2, inv_ai1, inv_ai2,
        impulse, world_r1, world_r2,
        &mut solver_data.total_linear_impulse,
    );
}

// --- Per-axis linear DOF solver ---

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

    let inv_mass1 = inertia1.effective_inv_mass();
    let inv_mass2 = inertia2.effective_inv_mass();
    let inv_ai1 = inertia1.effective_inv_angular_inertia();
    let inv_ai2 = inertia2.effective_inv_angular_inertia();
    let r1_cross_n = world_r1.cross(axis_world);
    let r2_cross_n = world_r2.cross(axis_world);
    let w1 = inv_mass1.max_element() + r1_cross_n.dot(inv_ai1 * r1_cross_n);
    let w2 = inv_mass2.max_element() + r2_cross_n.dot(inv_ai2 * r2_cross_n);
    let compliance = joint.linear_compliance[axis_index];

    match joint.linear_motion[axis_index] {
        JointAxisMotion::Free => {}
        JointAxisMotion::Locked => {
            if value.abs() <= Scalar::EPSILON {
                return;
            }
            let lambda = &mut solver_data.linear_lock_lambda[axis_index];
            let dl = compute_lagrange_update(*lambda, value, &[w1, w2], compliance, dt);
            *lambda += dl;
            // Negate: XPBD gradient for body1 is -axis.
            apply_positional_impulse_d6(
                body1, body2, inv_mass1, inv_mass2, inv_ai1, inv_ai2,
                -dl * axis_world, world_r1, world_r2,
                &mut solver_data.total_linear_impulse,
            );
        }
        JointAxisMotion::Limited => {
            let limit = joint.linear_limits[axis_index];
            let lower = &mut solver_data.linear_lower_lambda[axis_index];
            let upper = &mut solver_data.linear_upper_lambda[axis_index];

            if value < limit.min {
                // Lower bound violated. Need to INCREASE value toward min.
                // Lock convention: impulse = -dl * axis, and dl < 0 for positive value
                //   → positive impulse → decreases value. So to INCREASE value,
                //   we need negative impulse = -applied * axis.
                let penetration = limit.min - value;
                let old = *lower;
                let dl = compute_lagrange_update(*lower, -penetration, &[w1, w2], compliance, dt);
                *lower = (*lower + dl).max(0.0);
                let applied = *lower - old;
                apply_positional_impulse_d6(
                    body1, body2, inv_mass1, inv_mass2, inv_ai1, inv_ai2,
                    -applied * axis_world, world_r1, world_r2,
                    &mut solver_data.total_linear_impulse,
                );
                *upper = 0.0;
            } else if value > limit.max {
                // Upper bound violated. Need to DECREASE value toward max.
                // To decrease value, need positive impulse = +applied * axis.
                let penetration = value - limit.max;
                let old = *upper;
                let dl = compute_lagrange_update(*upper, -penetration, &[w1, w2], compliance, dt);
                *upper = (*upper + dl).max(0.0);
                let applied = *upper - old;
                apply_positional_impulse_d6(
                    body1, body2, inv_mass1, inv_mass2, inv_ai1, inv_ai2,
                    applied * axis_world, world_r1, world_r2,
                    &mut solver_data.total_linear_impulse,
                );
                *lower = 0.0;
            } else {
                *lower = 0.0;
                *upper = 0.0;
            }
        }
    }
}

// --- Impulse application ---

/// Applies an angular impulse to both bodies.
fn apply_angular_impulse_d6(
    body1: &mut SolverBody,
    body2: &mut SolverBody,
    inv_i1: SymmetricTensor,
    inv_i2: SymmetricTensor,
    impulse: Vector,
    total: &mut Vector,
) {
    *total += impulse;
    let dq1 = Quaternion::from_scaled_axis(inv_i1 * impulse);
    body1.delta_rotation.0 = dq1 * body1.delta_rotation.0;
    let dq2 = Quaternion::from_scaled_axis(inv_i2 * (-impulse));
    body2.delta_rotation.0 = dq2 * body2.delta_rotation.0;
}

/// Applies a positional impulse to both bodies (position + rotation from lever arm).
#[allow(clippy::too_many_arguments)]
fn apply_positional_impulse_d6(
    body1: &mut SolverBody,
    body2: &mut SolverBody,
    inv_mass1: Vector,
    inv_mass2: Vector,
    inv_ai1: SymmetricTensor,
    inv_ai2: SymmetricTensor,
    impulse: Vector,
    r1: Vector,
    r2: Vector,
    total: &mut Vector,
) {
    *total += impulse;
    body1.delta_position += impulse * inv_mass1;
    body2.delta_position -= impulse * inv_mass2;
    let dq1 = Quaternion::from_scaled_axis(inv_ai1 * r1.cross(impulse));
    body1.delta_rotation.0 = dq1 * body1.delta_rotation.0;
    let dq2 = Quaternion::from_scaled_axis(inv_ai2 * r2.cross(-impulse));
    body2.delta_rotation.0 = dq2 * body2.delta_rotation.0;
}

/// Wraps an angle (radians) to `[-PI, PI]`.
#[inline]
fn wrap_to_pi(a: Scalar) -> Scalar {
    (a + PI).rem_euclid(TAU) - PI
}

/// Computes an effective "live" angular velocity for a body during the
/// joint solve, by adding the in-substep contribution of `delta_rotation`
/// (post-snapshot) onto the start-of-substep `angular_velocity`. Used by
/// the motor solver so that joint B sees joint A's in-substep corrections
/// when A and B share a body, instead of a stale ω.
///
/// `delta_rotation` is the body's current cumulative solver delta, and
/// `pre_solve` is the snapshot of that same field taken by the global
/// pre-solve system at the very start of this substep. Their ratio is the
/// rotation applied during this substep's solve phase so far.
#[inline]
fn live_angular_velocity(
    angular_velocity: Vector,
    delta_rotation: Quaternion,
    pre_solve: Quaternion,
    substep_dt: Scalar,
) -> Vector {
    let in_substep = delta_rotation * pre_solve.inverse();
    let mut w = 2.0 * Vector::new(in_substep.x, in_substep.y, in_substep.z) / substep_dt;
    if in_substep.w < 0.0 {
        w = -w;
    }
    angular_velocity + w
}

/// Decomposes a relative quaternion into `[twist, swing1, swing2]` angles
/// in true radians, following the right-hand rule for each axis:
///
/// - `twist` = rotation about local X
/// - `swing1` = rotation about local Y
/// - `swing2` = rotation about local Z
///
/// Used by the motor solver — the constraint and limit solvers use the
/// projected `-2*q.xyz()` approach instead because it has no singularities
/// and is numerically more stable in chains. Motors need true radians to
/// match the user-facing `target_position` semantics.
///
/// Near-π swing singularities exist but don't affect typical ragdoll joints.
fn decompose_twist_swing_angles(q: Quaternion) -> [Scalar; 3] {
    // Twist: project q onto the X axis.
    let twist_q = Quaternion::from_xyzw(q.x, 0.0, 0.0, q.w);
    let twist_q = if twist_q.length_squared() > Scalar::EPSILON {
        twist_q.normalize()
    } else {
        Quaternion::IDENTITY
    };
    let twist_angle = 2.0 * twist_q.x.atan2(twist_q.w);

    // Swing: everything that isn't twist.
    let swing_q = q * twist_q.inverse();
    let swung_x = swing_q * Vector::X;

    // For a rotation `+θ` about Y, glam rotates +X to (cos θ, 0, -sin θ),
    // so `-z` gives the positive-sense angle.
    let swing1 = (-swung_x.z).atan2(swung_x.x);
    // For a rotation `+θ` about Z, glam rotates +X to (cos θ, sin θ, 0),
    // so `+y` gives the positive-sense angle.
    let swing2 = swung_x.y.atan2(swung_x.x);

    [twist_angle, swing1, swing2]
}

// --- Angular motor solver ---

#[allow(clippy::too_many_arguments)]
fn solve_angular_motor(
    joint: &SixDofJoint,
    body1: &mut SolverBody,
    body2: &mut SolverBody,
    inertia1: &SolverBodyInertia,
    inertia2: &SolverBodyInertia,
    solver_data: &mut SixDofJointSolverData,
    axis_index: usize,
    dt: Scalar,
) {
    let motor = &joint.angular_motors[axis_index];

    let basis1 = current_basis1(body1, solver_data);
    let basis2 = current_basis2(body2, solver_data);

    // The motor's scalar DOF is one coordinate of the nonlinear twist/swing
    // decomposition of `q_rel = basis1.inverse() * basis2`. The naive "axis_world
    // = basis1 * X/Y/Z" direction is only the conjugate of that coordinate at the
    // reference pose. Away from it, the conjugate direction is the gradient
    //
    //     grad_i = d(decompose[i]) / d(ω_rel_in_basis1_frame),
    //
    // computed here by a 3-point forward difference. We then rotate grad_i to
    // world and use it as the effective joint axis for *everything*: effective
    // mass (w1/w2), velocity projection, and the correction impulse direction.
    // Using `axis_world` for those when `current_angle` lives in decomposed
    // coordinates was creating an indefinite feedback loop on shared-body chains
    // (spine_01 + spine_02 pump).
    let q_rel = basis1.inverse() * basis2;
    let current_angles = decompose_twist_swing_angles(q_rel);
    let current_angle = current_angles[axis_index];

    // Numerical Jacobian: dq_rel/dt = 0.5 * ω_rel_in_basis1 * q_rel (left multiply).
    const GRAD_EPS: Scalar = 1e-4;
    let mut grad_local = Vector::ZERO;
    for k in 0..3 {
        let probe_axis = match k {
            0 => Vector::X,
            1 => Vector::Y,
            _ => Vector::Z,
        };
        let dq = Quaternion::from_scaled_axis(probe_axis * GRAD_EPS);
        let q_probe = dq * q_rel;
        let a_probe = decompose_twist_swing_angles(q_probe)[axis_index];
        grad_local[k] = wrap_to_pi(a_probe - current_angle) / GRAD_EPS;
    }
    let grad_world = basis1 * grad_local;
    let grad_len_sq = grad_world.length_squared();
    if grad_len_sq <= Scalar::EPSILON {
        // Decomposition is locally degenerate (near a swing singularity).
        return;
    }

    let inv_i1 = inertia1.effective_inv_angular_inertia();
    let inv_i2 = inertia2.effective_inv_angular_inertia();
    let w1 = grad_world.dot(inv_i1 * grad_world);
    let w2 = grad_world.dot(inv_i2 * grad_world);
    let w_sum = w1 + w2;

    if w_sum <= Scalar::EPSILON {
        return;
    }

    // Use "live" intra-substep angular velocities rather than the stale
    // start-of-substep snapshot. When two motorized joints share a body,
    // the second joint's ω read needs to include the first joint's
    // in-substep corrections; otherwise there's a per-substep phase lag
    // that integrates into slow chain drift.
    let live_omega_1 = live_angular_velocity(
        body1.angular_velocity,
        body1.delta_rotation.0,
        solver_data.pre_solve_delta_rotation_1,
        dt,
    );
    let live_omega_2 = live_angular_velocity(
        body2.angular_velocity,
        body2.delta_rotation.0,
        solver_data.pre_solve_delta_rotation_2,
        dt,
    );
    // d(decompose[i])/dt expressed in the same coordinate as position_error.
    let measured_rate = (live_omega_2 - live_omega_1).dot(grad_world);
    let velocity_error = motor.target_velocity - measured_rate;

    // Both `current_angle` and `target_position` are in real radians.
    // Wrap the error to [-π, π] for shortest-path rotation.
    let position_error = wrap_to_pi(motor.target_position - current_angle);

    let Some(delta_lagrange) =
        compute_angular_motor_lagrange(velocity_error, position_error, w_sum, motor, dt)
    else {
        return;
    };

    solver_data.angular_motor_lagrange[axis_index] += delta_lagrange;

    // Correction along the conjugate direction of the scalar DOF, not basis1 * e_i.
    apply_angular_impulse_d6(
        body1,
        body2,
        inv_i1,
        inv_i2,
        -delta_lagrange * grad_world,
        &mut solver_data.total_angular_impulse,
    );
}

/// Computes the Lagrange multiplier delta for an angular motor.
/// Ported from RevoluteJoint's motor solver.
fn compute_angular_motor_lagrange(
    velocity_error: Scalar,
    position_error: Scalar,
    w_sum: Scalar,
    motor: &AngularMotor,
    dt: Scalar,
) -> Option<Scalar> {
    let target_velocity_change = match motor.motor_model {
        MotorModel::SpringDamper {
            frequency,
            damping_ratio,
        } => {
            let omega = TAU * frequency;
            let omega_sq = omega * omega;
            let two_zeta_omega = 2.0 * damping_ratio * omega;
            let inv_denominator = 1.0 / (1.0 + two_zeta_omega * dt + omega_sq * dt * dt);
            (omega_sq * position_error + two_zeta_omega * velocity_error) * dt * inv_denominator
        }
        MotorModel::AccelerationBased { stiffness, damping } => {
            damping * velocity_error + stiffness * position_error * dt
        }
        MotorModel::ForceBased { stiffness, damping } => {
            (stiffness * position_error + damping * velocity_error) * w_sum
        }
    };

    let correction = target_velocity_change * dt;
    if correction.abs() <= Scalar::EPSILON {
        return None;
    }

    let delta_lagrange = correction / w_sum;

    let delta_lagrange = if motor.max_torque < Scalar::MAX && motor.max_torque > 0.0 {
        let max_delta = motor.max_torque * dt * dt;
        delta_lagrange.clamp(-max_delta, max_delta)
    } else {
        delta_lagrange
    };

    Some(delta_lagrange)
}

impl PositionConstraint for SixDofJoint {}

impl AngularConstraint for SixDofJoint {}

/// Copies each [`SixDofJoint`]'s two bodies' [`PreSolveDeltaRotation`] into
/// the joint's [`SixDofJointSolverData`] at the start of every substep,
/// before the joint solve phase runs. The motor solver uses these to
/// derive a "live" intra-substep angular velocity — see
/// `solve_angular_motor`.
pub(crate) fn snapshot_six_dof_pre_solve_rotations(
    bodies: Query<&PreSolveDeltaRotation, Without<RigidBodyDisabled>>,
    mut joints: Query<
        (&SixDofJoint, &mut SixDofJointSolverData),
        (Without<RigidBody>, Without<JointDisabled>),
    >,
) {
    for (joint, mut solver_data) in &mut joints {
        let [e1, e2] = joint.entities();
        solver_data.pre_solve_delta_rotation_1 = bodies
            .get(e1)
            .map(|pre| pre.0 .0)
            .unwrap_or(Quaternion::IDENTITY);
        solver_data.pre_solve_delta_rotation_2 = bodies
            .get(e2)
            .map(|pre| pre.0 .0)
            .unwrap_or(Quaternion::IDENTITY);
    }
}
