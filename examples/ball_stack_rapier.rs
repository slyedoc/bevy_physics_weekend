/// Rapier copy of simple to campare
mod helper;

use bevy::prelude::*;
use bevy_inspector_egui::{ InspectorPlugin};
use bevy_rapier3d::prelude::*;


fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(helper::HelperPlugins)
        // Rapier
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        //.add_plugin(RapierRenderPlugin)
        .add_plugin(InspectorPlugin::<helper::BallStackConfig>::new())
        .add_startup_system(setup)
        .add_system(setup_level)
        .run();
}

fn setup_level(
    mut ev_reset: EventReader<helper::ResetEvent>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    stack_config: Res<helper::BallStackConfig>,
) {
    for _ in ev_reset.iter() {
        // spheres stack
        let mesh = meshes.add(Mesh::from(bevy::render::mesh::shape::UVSphere {
            radius: 1.0,
            sectors: stack_config.ball_sectors,
            stacks: stack_config.ball_stacks,
        }));

        for i in 0..stack_config.level_size {
            for j in 0..stack_config.level_size {
                for k in 0..stack_config.level_size {
                    let pos = Vec3::new(i as f32 * stack_config.offset, j as f32 * stack_config.offset, k as f32 * stack_config.offset);
                    commands
                        .spawn_bundle(PbrBundle {
                            transform: Transform::from_translation(pos),
                            mesh: mesh.clone(),
                            material: stack_config.ball_material.clone(),
                            ..Default::default()
                        })
                        .insert_bundle(RigidBodyBundle {
                            position: pos.into(),
                            velocity: RigidBodyVelocity {
                                linvel: Vec3::new(1.0, 2.0, 3.0).into(), // a little velocity to start a fall
                                angvel: Vec3::new(0.2, 0.0, 0.0).into(),
                            }
                            .into(),
                            ..RigidBodyBundle::default()
                        })
                        .insert_bundle(ColliderBundle {
                            shape: ColliderShape::ball(1.0).into(),
                            ..ColliderBundle::default()
                        })
                        //.insert(ColliderDebugRender::with_id(0))
                        .insert(ColliderPositionSync::Discrete)
                        .insert(helper::Reset)
                        .insert(Name::new("Sphere"));
                }
            }
        }
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
            transform: Transform::from_xyz(60.0, 60.0, 60.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..Default::default()
        })
        .insert(helper::CameraController::default())
        .insert(Name::new("Camera"));

    // light
    helper::spawn_light(&mut commands);

    /*
     * Ground
     */
    let ground_size = 200.1;
    let ground_height = 0.1;
    let ground_pos = Vec3::new(0.0, -ground_height, 0.0);
    commands
        .spawn()
        .insert_bundle(PbrBundle {
            transform: Transform::from_translation(ground_pos),
            mesh: meshes.add(Mesh::from(bevy::render::mesh::shape::Box {
                min_x: -ground_size,
                max_x: ground_size,
                min_y: -ground_height,
                max_y: ground_height,
                min_z: -ground_size,
                max_z: ground_size,
            })),
            material: materials.add(StandardMaterial {
                base_color: Color::GREEN,
                ..Default::default()
            }),
            ..Default::default()
        })
        .insert_bundle(ColliderBundle {
            shape: ColliderShape::cuboid(ground_size, ground_height, ground_size).into(),
            position: ground_pos.into(),
            ..ColliderBundle::default()
        })
        //.insert(ColliderDebugRender::default())
        .insert(ColliderPositionSync::Discrete)
        .insert(Name::new("Ground"));
}