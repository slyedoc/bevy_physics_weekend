/// Rapier Example
mod helper;
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use helper::*;

fn main() {
    App::new()
        .insert_resource(WindowDescriptor {
            title: "Rapier Stack".to_string(),
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
        stack_config.spawn(&mut commands, &mut meshes, helper::Engine::Rapier);
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
    let ground_size = 200.0;
    let ground_height = 10.0;
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
