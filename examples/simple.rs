mod helper;
use bevy::prelude::*;
use bevy_physics_weekend::prelude::*;
use bevy_inspector_egui::WorldInspectorPlugin;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(WorldInspectorPlugin::default())
        .add_plugin(PhysicsPlugin) // has to be after inspector plugin
        .add_plugin(helper::CameraControllerPlugin)
        .add_startup_system(setup)
        .run();
}

/// set up a simple 3D scene
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // sphere
    let sphere_radius = 1.0;
    commands
        .spawn_bundle(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::UVSphere { radius: sphere_radius, sectors: 100, stacks: 100 })),
        transform: Transform::from_xyz(0.0, 10.5, 0.0),
        material: materials.add(StandardMaterial {
             base_color: Color::RED,
             ..Default::default()
        }),
        ..Default::default()
    })
        .insert(Body {
            inv_mass: 1.0,
            shape: PyhsicsShape::Sphere{ radius: sphere_radius },
            elasticity: 0.5,
            ..Default::default()
        })
        .insert(Name::new("Sphere"));

    // Ground
    let ground_radius = 10.0;
    commands
        .spawn_bundle(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::UVSphere { radius: ground_radius, sectors: 100, stacks: 100 })),
        transform: Transform::from_xyz(0.0, -10.0, 0.0),
        material: materials.add(StandardMaterial {
             base_color: Color::GREEN,
             ..Default::default()
        }),
        ..Default::default()
    })
        .insert(Body {
            inv_mass: 0.0,
            shape: PyhsicsShape::Sphere{ radius: ground_radius },
            ..Default::default()
        })
        .insert(Name::new("Ground"));


    // camera
    commands
        .spawn_bundle(PerspectiveCameraBundle {
            transform: Transform::from_xyz(-2.0, 2.5, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..Default::default()
        })
        .insert(helper::CameraController::default())
        .insert(Name::new("Camera"));

    // light
    commands
        .spawn_bundle(DirectionalLightBundle {
            transform: Transform::from_xyz(0.0, 10.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
            directional_light: DirectionalLight {
                illuminance: 5000.,
                color: Color::WHITE,
                ..Default::default()
            },
            ..Default::default()
        })
        .insert(Name::new("Light"));
}
