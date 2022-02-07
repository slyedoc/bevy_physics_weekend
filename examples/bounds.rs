mod helper;

use bevy::prelude::*;
use bevy_physics_weekend::bounds::*;

fn main() {
    App::new()
        //.insert_resource(Msaa { samples: 2 })
        .add_plugins(DefaultPlugins)
        .add_plugin(helper::CameraControllerPlugin)
        .add_plugin(BoundingVolumePlugin::<sphere::BSphere>::default())
        .add_plugin(BoundingVolumePlugin::<aabb::Aabb>::default())
        .add_plugin(BoundingVolumePlugin::<obb::Obb>::default())
        .add_startup_system(setup)
        .add_system(rotation_system)
        .run();
}

#[derive(Component)]
struct Rotator;

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Camera
    commands.spawn_bundle(PerspectiveCameraBundle {
        transform: Transform::from_matrix(Mat4::face_toward(
            Vec3::new(0.1, 0.1, 1.0),
            Vec3::ZERO,
            Vec3::Y,
        )),
        ..Default::default()

    })
    .insert(helper::CameraController::default());

    // Light
    commands.spawn_bundle(PointLightBundle {
        transform: Transform::from_translation(Vec3::new(4.0, 8.0, 4.0)),
        ..Default::default()
    });

    // AABB
    commands
        .spawn_bundle(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Cube::default())),
            material: materials.add(Color::rgb(1.0, 1.0, 1.0).into()),
            transform: Transform::from_translation(Vec3::new(-1.5, 0.0, 0.0)),
            ..Default::default()
        })
        .insert(Bounded::<aabb::Aabb>::default())
        .insert(debug::DebugBounds)
        .insert(Rotator);
    // OBB
    commands
        .spawn_bundle(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Cube::default())),
            material: materials.add(Color::rgb(1.0, 1.0, 1.0).into()),
            transform: Transform::from_translation(Vec3::new(0.0, 0.0, 0.0)),
            ..Default::default()
        })
        .insert(Bounded::<obb::Obb>::default())
        .insert(debug::DebugBounds)
        .insert(Rotator);
    // Sphere
    commands
        .spawn_bundle(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Cube::default())),
            material: materials.add(Color::rgb(1.0, 1.0, 1.0).into()),
            transform: Transform::from_translation(Vec3::new(1.5, 0.0, 0.0)),
            ..Default::default()
        })
        .insert(Bounded::<sphere::BSphere>::default())
        .insert(debug::DebugBounds)
        .insert(Rotator);
    
}

/// Rotate the meshes to demonstrate how the bounding volumes update
fn rotation_system(time: Res<Time>, mut query: Query<&mut Transform, With<Rotator>>) {
    for mut transform in query.iter_mut() {
        let scale = Vec3::ONE * ((time.seconds_since_startup() as f32).sin() * 0.3 + 1.0) * 0.3;
        let rot_x = Quat::from_rotation_x((time.seconds_since_startup() as f32 / 5.0).sin() / 50.0);
        let rot_y = Quat::from_rotation_y((time.seconds_since_startup() as f32 / 3.0).sin() / 50.0);
        let rot_z = Quat::from_rotation_z((time.seconds_since_startup() as f32 / 4.0).sin() / 50.0);
        transform.scale = scale;
        transform.rotate(rot_x * rot_y * rot_z);
    }
}
