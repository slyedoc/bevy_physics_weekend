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

        // ball
        let mesh = meshes.add(Mesh::from(bevy::render::mesh::shape::UVSphere {
            radius: 1.0,
            sectors: 5,
            stacks: 5,
        }));

        commands
        .spawn_bundle(PbrBundle {
            transform: Transform::from_translation(pos),
            mesh: mesh.clone(),
            material: materials.add(StandardMaterial {
                base_color: Color::rgb(0.5, 0.5, 0.5),
                ..Default::default()
            }),
            ..Default::default()
        })
        .insert(RigidBody {
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