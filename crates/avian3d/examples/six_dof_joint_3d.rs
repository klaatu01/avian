//! An example demonstrating the [`SixDofJoint`] with various configurations.
//!
//! The scene shows four joints:
//! 1. All axes locked (equivalent to a FixedJoint) — top left
//! 2. All angular free (equivalent to a SphericalJoint) — top right
//! 3. Limited twist about X, swing locked (like a RevoluteJoint with limits) — bottom left
//! 4. Linear Y limited, others locked (like a PrismaticJoint along Y) — bottom right

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
        .insert_resource(SubstepCount(50))
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let cube_mesh = meshes.add(Cuboid::default());
    let sphere_mesh = meshes.add(Sphere::new(0.5));

    let anchor_material = materials.add(Color::srgb(0.6, 0.6, 0.7));
    let dynamic_material = materials.add(Color::srgb(0.8, 0.5, 0.3));
    let limited_material = materials.add(Color::srgb(0.3, 0.8, 0.5));
    let prismatic_material = materials.add(Color::srgb(0.3, 0.5, 0.8));

    let spacing = 4.0;

    // --- 1. All Locked (like FixedJoint) — top left ---
    {
        let pos = Vector::new(-spacing, spacing, 0.0);

        let anchor = commands
            .spawn((
                Mesh3d(cube_mesh.clone()),
                MeshMaterial3d(anchor_material.clone()),
                Transform::from_translation(pos.f32()),
                RigidBody::Kinematic,
                AngularVelocity(Vector::Z * 0.5),
            ))
            .id();

        let body = commands
            .spawn((
                Mesh3d(cube_mesh.clone()),
                MeshMaterial3d(dynamic_material.clone()),
                Transform::from_translation((pos + Vector::X * 2.0).f32()),
                RigidBody::Dynamic,
                MassPropertiesBundle::from_shape(&Cuboid::from_length(1.0), 1.0),
            ))
            .id();

        // All locked = FixedJoint equivalent.
        commands.spawn(
            SixDofJoint::new(anchor, body).with_local_anchor1(Vector::X * 2.0),
        );
    }

    // --- 2. Angular Free (like SphericalJoint) — top right ---
    {
        let pos = Vector::new(spacing, spacing, 0.0);

        let anchor = commands
            .spawn((
                Mesh3d(sphere_mesh.clone()),
                MeshMaterial3d(anchor_material.clone()),
                Transform::from_translation(pos.f32()),
                RigidBody::Static,
            ))
            .id();

        let body = commands
            .spawn((
                Mesh3d(cube_mesh.clone()),
                MeshMaterial3d(dynamic_material.clone()),
                Transform::from_translation((pos + Vector::new(0.0, -2.5, 0.0)).f32()),
                RigidBody::Dynamic,
                MassPropertiesBundle::from_shape(&Cuboid::from_length(1.0), 1.0),
                LinearVelocity(Vector::new(3.0, 0.0, 2.0)),
            ))
            .id();

        // All linear locked, all angular free = spherical pendulum.
        // Body hangs below with initial push so it swings.
        commands.spawn((
            SixDofJoint::new(anchor, body)
                .with_local_anchor2(Vector::new(0.0, 2.5, 0.0))
                .with_all_angular(JointAxisMotion::Free),
            JointDamping {
                linear: 0.0,
                angular: 0.5,
            },
        ));
    }

    // --- 3. Limited Twist (like limited RevoluteJoint) — bottom left ---
    {
        let pos = Vector::new(-spacing, -spacing, 0.0);

        let anchor = commands
            .spawn((
                Mesh3d(cube_mesh.clone()),
                MeshMaterial3d(anchor_material.clone()),
                Transform::from_translation(pos.f32()),
                RigidBody::Static,
            ))
            .id();

        let body = commands
            .spawn((
                Mesh3d(cube_mesh.clone()),
                MeshMaterial3d(limited_material.clone()),
                Transform::from_translation((pos + Vector::new(0.0, 0.0, 2.0)).f32()),
                RigidBody::Dynamic,
                MassPropertiesBundle::from_shape(&Cuboid::from_length(1.0), 1.0),
            ))
            .id();

        // Twist about local X is limited to ±45°.
        // The body's center of mass is offset from the pivot along world Z,
        // so gravity produces torque around world X and the body swings
        // within the twist limits.
        commands.spawn(
            SixDofJoint::new(anchor, body)
                .with_local_anchor2(Vector::new(0.0, 0.0, -2.0))
                .with_twist_limits(-PI / 4.0, PI / 4.0),
        );
    }

    // --- 4. Linear Y Limited (like PrismaticJoint along Y) — bottom right ---
    {
        let pos = Vector::new(spacing, -spacing, 0.0);

        let anchor = commands
            .spawn((
                Mesh3d(cube_mesh.clone()),
                MeshMaterial3d(anchor_material.clone()),
                Transform::from_translation(pos.f32()),
                RigidBody::Static,
            ))
            .id();

        let body = commands
            .spawn((
                Mesh3d(cube_mesh.clone()),
                MeshMaterial3d(prismatic_material),
                Transform::from_translation((pos + Vector::new(0.0, -1.5, 0.0)).f32()),
                RigidBody::Dynamic,
                MassPropertiesBundle::from_shape(&Cuboid::from_length(1.0), 1.0),
            ))
            .id();

        // Only Y translation is limited; all angular + X/Z translation locked.
        commands.spawn(
            SixDofJoint::new(anchor, body)
                .with_linear_y_limits(-3.0, 0.0),
        );
    }

    // Light
    commands.spawn((
        DirectionalLight {
            illuminance: 2000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::default().looking_at(Vec3::new(-1.0, -2.5, -1.5), Vec3::Y),
    ));

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 0.0, 18.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}
