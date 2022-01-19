mod helper;
use bevy::prelude::*;
use bevy_inspector_egui::InspectorPlugin;
use bevy_physics_weekend::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(helper::HelperPlugins)
        // our plugin
        .add_plugin(PhysicsPlugin)
        .add_plugin(InspectorPlugin::<helper::BallStackConfig>::new()) // has to be after inspector plugin
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
        info!("Reset");

        // Setup level
        let mesh = meshes.add(Mesh::from(shape::UVSphere {
            radius: 1.0,
            sectors: stack_config.ball_sectors,
            stacks: stack_config.ball_stacks,
        }));

        for i in 0..stack_config.level_size {
            for j in 0..stack_config.level_size {
                for k in 0..stack_config.level_size {
                    let pos = Vec3::new(
                        i as f32 * stack_config.offset,
                        j as f32 * stack_config.offset,
                        k as f32 * stack_config.offset,
                    );
                    commands
                        .spawn_bundle(PbrBundle {
                            transform: Transform::from_translation(pos),
                            mesh: mesh.clone(),
                            material: stack_config.ball_material.clone(),
                            ..Default::default()
                        })
                        .insert(Body {
                            collider: Collider::from(Sphere { radius: 1.0 }),
                            inv_mass: 1.0,
                            elasticity: 1.0,
                            friction: 0.5,
                            ..Default::default()
                        })
                        .insert(helper::Reset)
                        .insert(Name::new("Sphere"));
                }
            }
        }
    }
}
/// set up a simple 3D scene
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

    // Ground
    let ground_radius = 1000.0;
    commands
        .spawn_bundle(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::UVSphere {
                radius: ground_radius,
                sectors: 60,
                stacks: 60,
            })),
            transform: Transform::from_xyz(0.0, -ground_radius, 0.0),
            material: materials.add(StandardMaterial {
                base_color: Color::GREEN,
                ..Default::default()
            }),
            ..Default::default()
        })
        .insert(Body {
            inv_mass: 0.0,
            collider: Collider::from(Sphere {
                radius: ground_radius,
            }),
            friction: 0.5,
            elasticity: 0.5,
            ..Default::default()
        })
        .insert(Name::new("Ground"));
}
