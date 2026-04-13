//! A chain of bodies connected by spherical [`SixDofJoint`]s,
//! demonstrating multi-body dynamics with the D6 joint.
//!
//! The chain hangs from a static anchor and swings freely under gravity.

use avian3d::{math::*, prelude::*};
use bevy::prelude::*;
use examples_common_3d::ExampleCommonPlugin;

const CHAIN_LENGTH: usize = 8;
const LINK_SIZE: Scalar = 0.4;
const LINK_GAP: Scalar = 0.1;
const LINK_SPACING: Scalar = LINK_SIZE + LINK_GAP;

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
    let link_mesh = meshes.add(Cuboid::new(LINK_SIZE as f32, LINK_SIZE as f32, LINK_SIZE as f32));

    let colors = [
        Color::srgb(0.9, 0.3, 0.3),
        Color::srgb(0.9, 0.6, 0.2),
        Color::srgb(0.9, 0.9, 0.2),
        Color::srgb(0.3, 0.9, 0.3),
        Color::srgb(0.2, 0.7, 0.9),
        Color::srgb(0.4, 0.3, 0.9),
        Color::srgb(0.8, 0.3, 0.8),
        Color::srgb(0.6, 0.6, 0.6),
    ];

    // Static anchor at the top.
    let anchor = commands
        .spawn((
            Mesh3d(meshes.add(Sphere::new(0.3))),
            MeshMaterial3d(materials.add(Color::WHITE)),
            Transform::from_translation(Vec3::new(0.0, 3.0, 0.0)),
            RigidBody::Static,
        ))
        .id();

    let mut prev = anchor;

    for i in 0..CHAIN_LENGTH {
        let y = 3.0 - LINK_SPACING / 2.0 - (i as Scalar) * LINK_SPACING;

        let link = commands
            .spawn((
                Mesh3d(link_mesh.clone()),
                MeshMaterial3d(materials.add(colors[i % colors.len()])),
                Transform::from_translation(Vec3::new(0.0, y as f32, 0.0)),
                RigidBody::Dynamic,
                MassPropertiesBundle::from_shape(
                    &Cuboid::new(LINK_SIZE, LINK_SIZE, LINK_SIZE),
                    1.0,
                ),
            ))
            .id();

        // Connect with a spherical D6 joint (all angular free, all linear locked).
        commands.spawn((
            SixDofJoint::new(prev, link)
                .with_local_anchor1(if i == 0 {
                    // First link attaches at the anchor's center.
                    Vector::ZERO
                } else {
                    Vector::new(0.0, -LINK_SPACING / 2.0, 0.0)
                })
                .with_local_anchor2(Vector::new(0.0, LINK_SPACING / 2.0, 0.0))
                .with_all_angular(JointAxisMotion::Free),
            JointDamping {
                linear: 0.0,
                angular: 0.3,
            },
        ));

        prev = link;
    }

    // Give the bottom link a push to start it swinging.
    commands.entity(prev).insert(LinearVelocity(Vector::new(5.0, 0.0, 3.0)));

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
        Transform::from_xyz(0.0, 1.0, 12.0).looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
    ));
}
