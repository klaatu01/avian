//! A ragdoll built from [`SixDofJoint`]s.
//!
//! The pelvis starts kinematic — use Arrow Keys / WASD to move it.
//! Press Space to launch upward. Press R to release as dynamic (ragdoll mode).

use avian3d::{math::*, prelude::*};
use bevy::prelude::*;
use examples_common_3d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            PhysicsPlugins::default(),
            PhysicsDebugPlugin,
        ))
        .insert_resource(SubstepCount(12))
        .insert_resource(Gravity(Vector::NEG_Y * 9.81))
        .add_systems(Startup, setup)
        .add_systems(Update, (move_pelvis, release_pelvis))
        .run();
}

// Body part dimensions.
const PELVIS_HALF: Vector = Vector::new(0.2, 0.15, 0.1);
const TORSO_HALF: Vector = Vector::new(0.2, 0.3, 0.1);
const HEAD_RADIUS: Scalar = 0.14;
const UPPER_ARM_HALF: Vector = Vector::new(0.05, 0.17, 0.05);
const LOWER_ARM_HALF: Vector = Vector::new(0.04, 0.15, 0.04);
const UPPER_LEG_HALF: Vector = Vector::new(0.08, 0.22, 0.08);
const LOWER_LEG_HALF: Vector = Vector::new(0.06, 0.2, 0.06);
const FOOT_HALF: Vector = Vector::new(0.06, 0.04, 0.12);

// Approximate segment masses (kg) for a ~70 kg person.
const PELVIS_MASS: Scalar = 8.5;
const TORSO_MASS: Scalar = 14.0;
const HEAD_MASS: Scalar = 4.5;
const UPPER_ARM_MASS: Scalar = 2.0;
const LOWER_ARM_MASS: Scalar = 1.2;
const UPPER_LEG_MASS: Scalar = 7.0;
const LOWER_LEG_MASS: Scalar = 3.5;
const FOOT_MASS: Scalar = 1.0;

#[derive(Component)]
struct PelvisControl;

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Ground.
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(20.0, 0.1, 20.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.3, 0.3))),
        Transform::from_translation(Vec3::new(0.0, -2.0, 0.0)),
        RigidBody::Static,
        Collider::cuboid(20.0, 0.1, 20.0),
    ));

    let pants = materials.add(Color::srgb(0.25, 0.25, 0.35));
    let shirt = materials.add(Color::srgb(0.3, 0.45, 0.7));
    let skin = materials.add(Color::srgb(0.85, 0.7, 0.55));
    let base_y: Scalar = 1.5;

    // === Pelvis (kinematic root) ===
    let pelvis = spawn_box(
        &mut commands,
        &mut meshes,
        pants.clone(),
        PELVIS_HALF,
        Vector::new(0.0, base_y, 0.0),
        PELVIS_MASS,
        true,
    );
    commands.entity(pelvis).insert(PelvisControl);

    // === Torso ===
    let torso_y = base_y + PELVIS_HALF.y + TORSO_HALF.y;
    let torso = spawn_box(
        &mut commands,
        &mut meshes,
        shirt.clone(),
        TORSO_HALF,
        Vector::new(0.0, torso_y, 0.0),
        TORSO_MASS,
        false,
    );

    // Spine: limited all axes, stiff.
    commands.spawn((
        SixDofJoint::new(pelvis, torso)
            .with_local_anchor1(Vector::new(0.0, PELVIS_HALF.y, 0.0))
            .with_local_anchor2(Vector::new(0.0, -TORSO_HALF.y, 0.0))
            .with_twist_limits(-PI / 6.0, PI / 6.0)
            .with_swing1_limits(-PI / 8.0, PI / 8.0)
            .with_swing2_limits(-PI / 6.0, PI / 6.0),
        JointCollisionDisabled,
        JointDamping {
            linear: 0.0,
            angular: 3.0,
        },
    ));

    // === Head ===
    let head_y = torso_y + TORSO_HALF.y + HEAD_RADIUS;
    let head = commands
        .spawn((
            Mesh3d(meshes.add(Sphere::new(HEAD_RADIUS as f32))),
            MeshMaterial3d(skin.clone()),
            Transform::from_translation(Vector::new(0.0, head_y, 0.0).f32()),
            RigidBody::Dynamic,
            MassPropertiesBundle::from_shape(&Sphere::new(HEAD_RADIUS), HEAD_MASS),
            Collider::sphere(HEAD_RADIUS),
        ))
        .id();

    // Neck: limited twist + swing.
    commands.spawn((
        SixDofJoint::new(torso, head)
            .with_local_anchor1(Vector::new(0.0, TORSO_HALF.y, 0.0))
            .with_local_anchor2(Vector::new(0.0, -HEAD_RADIUS * 0.5, 0.0))
            .with_twist_limits(-PI / 4.0, PI / 4.0)
            .with_swing1_limits(-PI / 6.0, PI / 6.0)
            .with_swing2_limits(-PI / 6.0, PI / 6.0),
        JointCollisionDisabled,
        JointDamping {
            linear: 0.0,
            angular: 2.0,
        },
    ));

    // === Arms ===
    for side in [-1.0_f64, 1.0] {
        let side = side as Scalar;
        let shoulder_x = side * (TORSO_HALF.x + UPPER_ARM_HALF.x);

        // Upper arm.
        let upper_arm_y = torso_y + TORSO_HALF.y - UPPER_ARM_HALF.y * 0.5;
        let upper_arm = spawn_box(
            &mut commands,
            &mut meshes,
            shirt.clone(),
            UPPER_ARM_HALF,
            Vector::new(shoulder_x, upper_arm_y, 0.0),
            UPPER_ARM_MASS,
            false,
        );

        // Shoulder: ball-and-socket with limits.
        commands.spawn((
            SixDofJoint::new(torso, upper_arm)
                .with_local_anchor1(Vector::new(side * TORSO_HALF.x, TORSO_HALF.y, 0.0))
                .with_local_anchor2(Vector::new(0.0, UPPER_ARM_HALF.y, 0.0))
                .with_twist_limits(-PI / 2.0, PI / 2.0)
                .with_swing1_limits(-PI / 2.0, PI / 2.0)
                .with_swing2_limits(-PI / 3.0, PI / 2.0),
            JointCollisionDisabled,
            JointDamping {
                linear: 0.0,
                angular: 1.0,
            },
        ));

        // Forearm.
        let lower_arm_y = upper_arm_y - UPPER_ARM_HALF.y - LOWER_ARM_HALF.y;
        let lower_arm = spawn_box(
            &mut commands,
            &mut meshes,
            skin.clone(),
            LOWER_ARM_HALF,
            Vector::new(shoulder_x, lower_arm_y, 0.0),
            LOWER_ARM_MASS,
            false,
        );

        // Elbow: hinge about X (bend forward/backward).
        commands.spawn((
            SixDofJoint::new(upper_arm, lower_arm)
                .with_local_anchor1(Vector::new(0.0, -UPPER_ARM_HALF.y, 0.0))
                .with_local_anchor2(Vector::new(0.0, LOWER_ARM_HALF.y, 0.0))
                .with_twist_limits(0.0, 2.5),
            JointCollisionDisabled,
            JointDamping {
                linear: 0.0,
                angular: 0.5,
            },
        ));
    }

    // === Legs ===
    for side in [-1.0_f64, 1.0] {
        let side = side as Scalar;
        let hip_x = side * (PELVIS_HALF.x * 0.6);

        // Thigh.
        let upper_leg_y = base_y - PELVIS_HALF.y - UPPER_LEG_HALF.y;
        let upper_leg = spawn_box(
            &mut commands,
            &mut meshes,
            skin.clone(),
            UPPER_LEG_HALF,
            Vector::new(hip_x, upper_leg_y, 0.0),
            UPPER_LEG_MASS,
            false,
        );

        // Hip: ball-and-socket with limits.
        commands.spawn((
            SixDofJoint::new(pelvis, upper_leg)
                .with_local_anchor1(Vector::new(hip_x, -PELVIS_HALF.y, 0.0))
                .with_local_anchor2(Vector::new(0.0, UPPER_LEG_HALF.y, 0.0))
                .with_twist_limits(-PI / 6.0, PI / 6.0)
                .with_swing1_limits(-PI / 6.0, PI / 3.0)
                .with_swing2_limits(-PI / 4.0, PI / 4.0),
            JointCollisionDisabled,
            JointDamping {
                linear: 0.0,
                angular: 1.5,
            },
        ));

        // Shin.
        let lower_leg_y = upper_leg_y - UPPER_LEG_HALF.y - LOWER_LEG_HALF.y;
        let lower_leg = spawn_box(
            &mut commands,
            &mut meshes,
            skin.clone(),
            LOWER_LEG_HALF,
            Vector::new(hip_x, lower_leg_y, 0.0),
            LOWER_LEG_MASS,
            false,
        );

        // Knee: hinge about X (bend backward).
        commands.spawn((
            SixDofJoint::new(upper_leg, lower_leg)
                .with_local_anchor1(Vector::new(0.0, -UPPER_LEG_HALF.y, 0.0))
                .with_local_anchor2(Vector::new(0.0, LOWER_LEG_HALF.y, 0.0))
                .with_twist_limits(0.0, 2.5),
            JointCollisionDisabled,
            JointDamping {
                linear: 0.0,
                angular: 0.5,
            },
        ));

        // Foot.
        let foot_y = lower_leg_y - LOWER_LEG_HALF.y - FOOT_HALF.y;
        let foot_z = FOOT_HALF.z - LOWER_LEG_HALF.z;
        let foot_size = FOOT_HALF * 2.0;
        let foot = commands
            .spawn((
                Mesh3d(meshes.add(Cuboid::new(
                    foot_size.x as f32,
                    foot_size.y as f32,
                    foot_size.z as f32,
                ))),
                MeshMaterial3d(pants.clone()),
                Transform::from_translation(Vector::new(hip_x, foot_y, foot_z).f32()),
                RigidBody::Dynamic,
                MassPropertiesBundle::from_shape(
                    &Cuboid::new(foot_size.x, foot_size.y, foot_size.z),
                    FOOT_MASS,
                ),
                Collider::cuboid(foot_size.x, foot_size.y, foot_size.z),
            ))
            .id();

        // Ankle: limited flex/extend.
        commands.spawn((
            SixDofJoint::new(lower_leg, foot)
                .with_local_anchor1(Vector::new(0.0, -LOWER_LEG_HALF.y, 0.0))
                .with_local_anchor2(Vector::new(0.0, FOOT_HALF.y, -foot_z))
                .with_twist_limits(-PI / 8.0, PI / 8.0)
                .with_swing1_limits(-PI / 6.0, PI / 4.0)
                .with_swing2_limits(-PI / 8.0, PI / 8.0),
            JointCollisionDisabled,
            JointDamping {
                linear: 0.0,
                angular: 2.0,
            },
        ));
    }

    // Light
    commands.spawn((
        DirectionalLight {
            illuminance: 3000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::default().looking_at(Vec3::new(-1.0, -2.5, -1.5), Vec3::Y),
    ));

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 1.5, 5.0).looking_at(Vec3::new(0.0, 0.5, 0.0), Vec3::Y),
    ));
}

/// Spawns a box-shaped body part with collider.
fn spawn_box(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    material: Handle<StandardMaterial>,
    half_extents: Vector,
    position: Vector,
    mass: Scalar,
    kinematic: bool,
) -> Entity {
    let size = half_extents * 2.0;
    let mut ec = commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(size.x as f32, size.y as f32, size.z as f32))),
        MeshMaterial3d(material),
        Transform::from_translation(position.f32()),
        Collider::cuboid(size.x, size.y, size.z),
    ));
    if kinematic {
        ec.insert(RigidBody::Kinematic);
    } else {
        ec.insert((
            RigidBody::Dynamic,
            MassPropertiesBundle::from_shape(&Cuboid::new(size.x, size.y, size.z), mass),
        ));
    }
    ec.id()
}

/// Move the kinematic pelvis with WASD / arrows. Space = jump.
fn move_pelvis(
    time: Res<Time>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut query: Query<&mut LinearVelocity, With<PelvisControl>>,
) {
    let speed = 4.0;
    let dt = time.delta_secs_f64().adjust_precision();

    for mut vel in &mut query {
        let up = keyboard.any_pressed([KeyCode::KeyW, KeyCode::ArrowUp]);
        let down = keyboard.any_pressed([KeyCode::KeyS, KeyCode::ArrowDown]);
        let left = keyboard.any_pressed([KeyCode::KeyA, KeyCode::ArrowLeft]);
        let right = keyboard.any_pressed([KeyCode::KeyD, KeyCode::ArrowRight]);

        let horizontal = right as i8 - left as i8;
        let vertical = up as i8 - down as i8;

        vel.x += horizontal as Scalar * speed * dt;
        vel.y += vertical as Scalar * speed * dt;

        if keyboard.just_pressed(KeyCode::Space) {
            vel.0 += Vector::new(0.0, 8.0, 0.0);
        }

        vel.0 *= 0.92;
    }
}

/// Press R to release the pelvis as a dynamic body (full ragdoll).
fn release_pelvis(
    mut commands: Commands,
    keyboard: Res<ButtonInput<KeyCode>>,
    query: Query<Entity, With<PelvisControl>>,
) {
    if keyboard.just_pressed(KeyCode::KeyR) {
        for entity in &query {
            commands.entity(entity).remove::<PelvisControl>();
            commands.entity(entity).insert((
                RigidBody::Dynamic,
                MassPropertiesBundle::from_shape(
                    &Cuboid::new(
                        PELVIS_HALF.x * 2.0,
                        PELVIS_HALF.y * 2.0,
                        PELVIS_HALF.z * 2.0,
                    ),
                    PELVIS_MASS,
                ),
            ));
        }
    }
}
