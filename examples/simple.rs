mod helper;
use bevy::prelude::*;
use bevy_inspector_egui::WorldInspectorPlugin;
use bevy_physics_weekend::prelude::*;

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
    asset_server: Res<AssetServer>,
) {
    let roundness = 60;

    // sphere

    let ball_material = materials.add(StandardMaterial {
        base_color_texture: Some(asset_server.load("checker_red.png")),
        ..Default::default()
    });
    let mesh = meshes.add(Mesh::from(shape::UVSphere {
        radius: 1.0,
        sectors: roundness,
        stacks: roundness,
    }));

    spawn_ball(
        Vec3::new(0.0, 10.0, 0.0),
        Vec3::new(0.0, 0.0, 0.0),
        &mut commands,
        mesh.clone(),
        ball_material.clone(),
    );

    spawn_ball(
        Vec3::new(3.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 0.0),
        &mut commands,
        mesh.clone(),
        ball_material.clone(),
    );

    let size = 5;
    for i in 0..size {
        for j in 0..size {
            for k in 0..size {
                spawn_ball(
                    Vec3::new(i as f32 * 2.2 , j as f32 * 2.2, k as f32 * 2.2),
                    Vec3::new(0.0, 0.0, 0.0),
                    &mut commands,
                    mesh.clone(),
                    ball_material.clone(),
                );
            }
        }
    }

    spawn_ball(
        Vec3::new(50.0, 1.0, 0.0),
        Vec3::new(-1000.0, 0.0, 0.0),
        &mut commands,
        mesh,
        ball_material,
    );

    // Ground
    let ground_radius = 1000.0;
    commands
        .spawn_bundle(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::UVSphere {
                radius: ground_radius,
                sectors: roundness,
                stacks: roundness,
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
            shape: PyhsicsShape::Sphere {
                radius: ground_radius,
            },
            friction: 0.1,
            elasticity: 0.5,
            ..Default::default()
        })
        .insert(Name::new("Ground"));

    // camera
    commands
        .spawn_bundle(PerspectiveCameraBundle {
            transform: Transform::from_xyz(30.0, 5.0, 30.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..Default::default()
        })
        .insert(helper::CameraController::default())
        .insert(Name::new("Camera"));

    // light
    commands
        .spawn_bundle(DirectionalLightBundle {
            transform: Transform::from_xyz(0.0, 10.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
            directional_light: DirectionalLight {
                shadows_enabled: true,
                illuminance: 5000.,
                color: Color::WHITE,
                ..Default::default()
            },
            ..Default::default()
        })
        .insert(Name::new("Light"));
}

fn spawn_ball(
    pos: Vec3,
    linear_velocity: Vec3,
    commands: &mut Commands,
    mesh: Handle<Mesh>,
    material: Handle<StandardMaterial>,
) {
    commands
        .spawn_bundle(PbrBundle {
            transform: Transform::from_translation(pos),
            mesh,
            material,
            ..Default::default()
        })
        .insert(Body {
            shape: PyhsicsShape::Sphere { radius: 1.0 },
            inv_mass: 1.0,
            elasticity: 1.0,
            friction: 0.5,
            linear_velocity,
            ..Default::default()
        })
        .insert(Name::new("Sphere"));
}
