/// Rapier Example
mod helper;
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use helper::*;

// TODO: Reset not working, see https://github.com/dimforge/bevy_rapier/issues/111
fn main() {
    App::new()
        .insert_resource(WindowDescriptor {
            title: "Rapier Ball Stack".to_string(),
            vsync: false,
            ..Default::default()
        })
        .add_plugins(DefaultPlugins)
        .add_plugins(HelperPlugins)
        .add_plugin(helper::StackConfigPlugin)
        // Rapier
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        //.add_plugin(RapierRenderPlugin)
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

            commands
                .spawn_bundle(PbrBundle {
                    transform: Transform::from_translation(pos),
                    mesh: mesh.clone(),
                    material: stack_config.ball_material.clone(),
                    ..Default::default()
                })
                .insert_bundle(RigidBodyBundle {
                    position: pos.into(),
                    // TODO: Remove this, a small impulse so its not a stable stack
                    velocity: RigidBodyVelocity {
                        linvel: Vec3::new(0.1, 0.0, 0.0).into(), // a little velocity to start a fall
                        angvel: Vec3::new(0.1, 0.0, 0.0).into(),
                    }
                    .into(),
                    ..Default::default()
                })
                .insert_bundle(ColliderBundle {
                    shape: match stack_config.stack_item {
                        StackItem::Sphere { radius, .. } => ColliderShape::ball(radius).into(),
                        StackItem::Box { size } => {
                            ColliderShape::cuboid(size.x / 2.0, size.y / 2.0, size.z / 2.0).into()
                        }
                    },
                    ..ColliderBundle::default()
                })
                //.insert(ColliderDebugRender::with_id(0))
                .insert(ColliderPositionSync::Discrete)
                .insert(helper::Reset)
                .insert(Name::new("Stack Item"));
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
