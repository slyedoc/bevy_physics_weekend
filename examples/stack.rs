mod helper;
use bevy::prelude::*;
use bevy_inspector_egui::InspectorPlugin;
use bevy_physics_weekend::{primitives::*, PhysicsConfig, PhysicsPlugin};

fn main() {
    App::new()
        .insert_resource(WindowDescriptor {
            title: "Physics Ball Stack".to_string(),
            vsync: false, // just for testing
            ..Default::default()
        })
        .add_plugins(DefaultPlugins)
        .add_plugins(helper::HelperPlugins)
        .add_plugin(helper::StackConfigPlugin)

        // our plugin
        .add_plugin(PhysicsPlugin)
        
        // Custom helpers
        .add_plugin(InspectorPlugin::<PhysicsConfig>::new())
        .add_startup_system(setup)
        .add_system(setup_level)
        .run();
}

fn setup_level(
    mut ev_reset: EventReader<helper::ResetEvent>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    stack_config: Res<helper::StackConfig>,
) {
    for _ in ev_reset.iter() {
        info!("Reset");

        // Setup level
        let mesh = stack_config.get_mesh(&mut meshes);

        let b2 = stack_config.base_size * stack_config.base_size;
        let mut pos = Vec3::new(0.0, stack_config.start_height, 0.0);
        for i in 0..stack_config.count {
            if i % stack_config.base_size == 0 {
                pos.x = 0.0;
                pos.z += stack_config.grid_offset;
            } else {
                pos.x += stack_config.grid_offset;
            }
            if i % b2 == 0 {
                pos.x = 0.0;
                pos.z = 0.0;
                pos.y += stack_config.grid_offset;
            }

            let e = commands
                .spawn_bundle(PbrBundle {
                    transform: Transform::from_translation(pos),
                    mesh: mesh.clone(),
                    material: stack_config.ball_material.clone(),
                    ..Default::default()
                })
                .insert(Body {
                    inv_mass: 1.0,
                    elasticity: 0.9,
                    friction: 0.5,
                    ..Default::default()
                })
                .insert(helper::Reset)
                .insert(Name::new("Stack Item"))
                .id();

            match stack_config.stack_item {
                helper::StackItem::Box { size } => {
                    commands.entity(e).insert(ColliderBox::from(size))
                }
                helper::StackItem::Sphere { radius, .. } => {
                    commands.entity(e).insert(ColliderSphere::new(radius))
                }
            };
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
            transform: Transform::from_xyz(0.0, -ground_radius - 1.0, 0.0),
            material: materials.add(StandardMaterial {
                base_color: Color::GREEN,
                ..Default::default()
            }),
            ..Default::default()
        })
        .insert(Body {
            inv_mass: 0.0,
            friction: 0.5,
            elasticity: 1.0,
            ..Default::default()
        })
        .insert(ColliderSphere::new(ground_radius))
        .insert(Name::new("Ground"));
}
