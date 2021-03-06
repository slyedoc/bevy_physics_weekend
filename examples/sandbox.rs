mod helper;
use bevy::prelude::*;
use bevy_physics_weekend::{
    colliders::{ColliderBox, ColliderSphere},
    debug::PhysicsDebugPlugin,
    primitives::Body,
    PhysicsPlugin,
};
use helper::HelperPlugin;

fn main() {
    App::new()
        .insert_resource(Msaa { samples: 4 })
        .insert_resource(WindowDescriptor {
            title: "Physics Sandbox".to_string(),
            vsync: false, // just for testing
            ..Default::default()
        })
        .add_plugins(DefaultPlugins)
        .add_plugin(HelperPlugin)
        // our plugin
        .add_plugin(PhysicsPlugin)
        .add_plugin(PhysicsDebugPlugin)
        .add_startup_system(setup)
        .add_system(setup_level)
        .run();
}

fn setup_level(
    mut ev_reset: EventReader<helper::ResetEvent>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for _ in ev_reset.iter() {
        info!("Reset");

        // Spheres
        for i in 0..1 {
            commands
                .spawn_bundle(PbrBundle {
                    transform: Transform::from_xyz(i as f32 * 2.0, 2.0, 0.0),
                    mesh: meshes.add(Mesh::from(bevy::render::mesh::shape::UVSphere {
                        radius: 1.0,
                        ..Default::default()
                    })),
                    material: materials.add(StandardMaterial {
                        base_color: Color::rgb(0.5, 0.5, 0.5),
                        ..Default::default()
                    }),
                    ..Default::default()
                })
                .insert(Body {
                    inv_mass: 1.0,
                    elasticity: 0.9,
                    friction: 0.5,
                    ..Default::default()
                })
                //.insert(Bounded::<aabb::Aabb>::default())
                .insert(ColliderSphere::new(1.0))
                .insert(helper::Reset)
                .insert(Name::new("Sphere"));
        }

        let shape_box = shape::Box::new(1.0, 1.0, 1.0);
        commands
            .spawn_bundle(PbrBundle {
                transform: Transform::from_xyz(0.0, 2.0, -5.0),
                mesh: meshes.add(shape_box.into()),
                material: materials.add(StandardMaterial {
                    base_color: Color::rgba(1.0, 1.0, 1.0, 0.5),
                    alpha_mode: AlphaMode::Blend,
                    ..Default::default()
                }),
                ..Default::default()
            })
            .insert(Body {
                inv_mass: 1.0,
                elasticity: 0.9,
                friction: 0.5,
                ..Default::default()
            })
            .insert(ColliderBox::from(Vec3::ONE))
            .insert(helper::Reset)
            .insert(Name::new("Box"));

        commands
            .spawn_bundle(PbrBundle {
                transform: Transform {
                    translation: Vec3::new(4.0, 3.0, -5.0),
                    rotation: Quat::from_euler(EulerRot::XYZ, 1.0, 1.0, 0.0),
                    ..Default::default()
                },
                mesh: meshes.add(shape_box.into()),
                material: materials.add(StandardMaterial {
                    base_color: Color::rgba(1.0, 1.0, 1.0, 0.5),
                    alpha_mode: AlphaMode::Blend,
                    ..Default::default()
                }),
                ..Default::default()
            })
            .insert(Body {
                inv_mass: 1.0,
                elasticity: 0.9,
                friction: 0.5,
                ..Default::default()
            })
            //.insert(Bounded::<aabb::Aabb>::default())
            .insert(ColliderBox::from(Vec3::ONE))
            .insert(helper::Reset)
            .insert(Name::new("Box"));
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // camera
    commands
        .spawn_bundle(PerspectiveCameraBundle {
            transform: Transform::from_xyz(5.0, 5.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..Default::default()
        })
        .insert(helper::CameraController::default())
        .insert(Name::new("Camera"));

    // light
    helper::spawn_light(&mut commands);

    //Ground
    commands
        .spawn_bundle(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Box::new(100.0, 1.0, 100.0))),
            transform: Transform::from_xyz(0.0, -0.5, 0.0),
            material: materials.add(StandardMaterial {
                base_color: Color::GREEN,
                ..Default::default()
            }),
            ..Default::default()
        })
        .insert(Body {
            inv_mass: 0.0,
            friction: 0.5,
            elasticity: 0.5,
            ..Default::default()
        })
        //.insert(Bounded::<aabb::Aabb>::default())
        .insert(ColliderBox::from(Vec3::new(100.0, 1.0, 100.0)))
        .insert(Name::new("Ground"));

    // /*
    //  * Ground
    //  */
    // let ground_size = 200.1;
    // let ground_height = 0.1;
    // let ground_pos = Vec3::new(0.0, -ground_height, 0.0);
    // let ground_box = shape::Box {
    //     min_x: -ground_size,
    //     max_x: ground_size,
    //     min_y: -ground_height,
    //     max_y: ground_height,
    //     min_z: -ground_size,
    //     max_z: ground_size,
    // };
    // commands
    //     .spawn()
    //     .insert_bundle(PbrBundle {
    //         transform: Transform::from_translation(ground_pos),
    //         mesh: meshes.add(ground_box.into()),
    //         material: materials.add(StandardMaterial {
    //             base_color: Color::GREEN,
    //             ..Default::default()
    //         }),
    //         ..Default::default()
    //     })
    //     .insert(Body {
    //         collider: ground_box.into(),
    //         inv_mass: 0.0,
    //         elasticity: 1.0,
    //         friction: 0.5,
    //         ..Default::default()
    //     })
    //     .insert(Name::new("Ground"));
}
