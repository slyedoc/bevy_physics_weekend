mod helper;
use bevy::prelude::*;
use bevy_physics_weekend::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(PhysicsPlugin)
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
    // cube
    commands
        .spawn_bundle(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Cube { size: 1.0 })),
        transform: Transform::from_xyz(0.0, 10.5, 0.0),
        material: materials.add(StandardMaterial {
             base_color: Color::RED,
             ..Default::default()
        }),
        ..Default::default()
    })
        .insert(Body::default())
        .insert(Name::new("Cube"));

    // Ground
    commands
        .spawn_bundle(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::UVSphere { radius: 1000.0, sectors: 100, stacks: 100 })),
        transform: Transform::from_xyz(0.0, -1000.0, 0.0),
        material: materials.add(StandardMaterial {
             base_color: Color::GREEN,
             ..Default::default()
        }),
        ..Default::default()
        // materials.add(CustomMaterial {
        //     color: Color::GREEN,
        // }),
    })
        .insert(Body {
            linear_velocity: Vec3::ZERO,
            inv_mass: 0.0,
            shape: PyhsicsShape::Sphere{ radius: 1000.0 },
        })
        .insert(Name::new("Sphere"));


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
