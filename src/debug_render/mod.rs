use bevy::prelude::*;
use bevy_polyline::*;

pub fn debug_render_system(
    mut commands: Commands,
    mut polylines: ResMut<Assets<Polyline>>,
    mut polyline_materials: ResMut<Assets<PolylineMaterial>>,
) {
    commands.spawn_bundle(PolylineBundle {
        polyline: polylines.add(Polyline {
            vertices: vec![
                Vec3::new(-0.5, -0.5, -0.5),
                Vec3::new(0.5, -0.5, -0.5),
                Vec3::new(0.5, 0.5, -0.5),
                Vec3::new(-0.5, 0.5, -0.5),
                Vec3::new(-0.5, 0.5, 0.5),
                Vec3::new(0.5, 0.5, 0.5),
                Vec3::new(0.5, -0.5, 0.5),
                Vec3::new(-0.5, -0.5, 0.5),
            ],
            ..Default::default()
        }),
        material: polyline_materials.add(PolylineMaterial {
            width: 3.0,
            color: Color::RED,
            perspective: false,
            ..Default::default()
        }),
        ..Default::default()
    });
}