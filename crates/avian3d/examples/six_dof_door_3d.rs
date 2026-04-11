//! A door on a hinge using a [`SixDofJoint`] with asymmetric twist limits.
//!
//! Demonstrates:
//! - Twist free with asymmetric limits (0° to 120°) acting as hinge stops
//! - A universal joint (cardan) that transmits rotation between misaligned shafts
//! - A sliding shelf on a rail with linear X limits

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
    let spacing = 5.0;

    // --- 1. Door with hinge stops (left) ---
    {
        let pos = Vector::new(-spacing, 0.0, 0.0);

        // Door frame (static post).
        let frame_mesh = meshes.add(Cuboid::new(0.2, 3.0, 0.2));
        let frame_material = materials.add(Color::srgb(0.4, 0.3, 0.2));

        let door_frame = commands
            .spawn((
                Mesh3d(frame_mesh),
                MeshMaterial3d(frame_material),
                Transform::from_translation(pos.f32()),
                RigidBody::Static,
            ))
            .id();

        // Door panel.
        let door_mesh = meshes.add(Cuboid::new(2.0, 2.8, 0.1));
        let door_material = materials.add(Color::srgb(0.6, 0.4, 0.2));

        let door = commands
            .spawn((
                Mesh3d(door_mesh),
                MeshMaterial3d(door_material),
                Transform::from_translation((pos + Vector::new(1.0, 0.0, 0.0)).f32()),
                RigidBody::Dynamic,
                MassPropertiesBundle::from_shape(&Cuboid::new(2.0, 2.8, 0.1), 5.0),
                AngularVelocity(Vector::Y * 3.0),
            ))
            .id();

        // Hinge along Y axis: orient basis so local X = world Y.
        // Twist (about local X = world Y) is limited to 0°..120°.
        // Everything else locked.
        commands.spawn((
            SixDofJoint::new(door_frame, door)
                .with_local_anchor2(Vector::new(-1.0, 0.0, 0.0))
                .with_basis(Quaternion::from_rotation_z(PI / 2.0))
                .with_twist_limits(0.0, 2.0 * PI / 3.0),
            JointDamping {
                linear: 0.0,
                angular: 1.0,
            },
        ));
    }

    // --- 2. Universal / cardan joint (center) ---
    {
        let pos = Vector::new(0.0, 1.0, 0.0);

        // Driving shaft (kinematic, rotating).
        let shaft_mesh = meshes.add(Cylinder::new(0.15, 2.0));
        let shaft_material = materials.add(Color::srgb(0.7, 0.7, 0.8));

        let shaft1 = commands
            .spawn((
                Mesh3d(shaft_mesh.clone()),
                MeshMaterial3d(shaft_material.clone()),
                Transform::from_translation(pos.f32()),
                RigidBody::Kinematic,
                AngularVelocity(Vector::Y * 2.0),
            ))
            .id();

        // Driven shaft (dynamic), offset and angled.
        let shaft2_pos = pos + Vector::new(0.0, -2.5, 0.0);
        let shaft2 = commands
            .spawn((
                Mesh3d(shaft_mesh),
                MeshMaterial3d(shaft_material),
                Transform::from_translation(shaft2_pos.f32())
                    .with_rotation(Quat::from_rotation_z(0.4)),
                RigidBody::Dynamic,
                MassPropertiesBundle::from_shape(&Cylinder::new(0.15, 2.0), 2.0),
            ))
            .id();

        // Universal joint: twist locked (no axial spin), both swings free.
        // This transmits rotation while allowing angular misalignment.
        commands.spawn(
            SixDofJoint::new(shaft1, shaft2)
                .with_local_anchor1(Vector::new(0.0, -1.0, 0.0))
                .with_local_anchor2(Vector::new(0.0, 1.0, 0.0))
                .with_swing1(JointAxisMotion::Free)
                .with_swing2(JointAxisMotion::Free),
        );
    }

    // --- 3. Sliding shelf on a rail (right) ---
    {
        let pos = Vector::new(spacing, 0.0, 0.0);

        // Rail (static).
        let rail_mesh = meshes.add(Cuboid::new(4.0, 0.1, 0.1));
        let rail_material = materials.add(Color::srgb(0.5, 0.5, 0.5));

        let rail = commands
            .spawn((
                Mesh3d(rail_mesh),
                MeshMaterial3d(rail_material),
                Transform::from_translation(pos.f32()),
                RigidBody::Static,
            ))
            .id();

        // Shelf (dynamic).
        let shelf_mesh = meshes.add(Cuboid::new(1.5, 0.2, 1.0));
        let shelf_material = materials.add(Color::srgb(0.8, 0.6, 0.3));

        let shelf = commands
            .spawn((
                Mesh3d(shelf_mesh),
                MeshMaterial3d(shelf_material),
                Transform::from_translation((pos + Vector::new(0.0, -0.15, 0.0)).f32()),
                RigidBody::Dynamic,
                MassPropertiesBundle::from_shape(&Cuboid::new(1.5, 0.2, 1.0), 2.0),
                LinearVelocity(Vector::new(3.0, 0.0, 0.0)),
            ))
            .id();

        // Slide along X with limits, everything else locked.
        // The shelf can slide ±1.5 units along the rail.
        commands.spawn(
            SixDofJoint::new(rail, shelf)
                .with_linear_x_limits(-1.5, 1.5),
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
        Transform::from_xyz(0.0, 2.0, 14.0).looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
    ));
}
