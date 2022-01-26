use bevy::prelude::*;
use bevy_inspector_egui::Inspectable;
use bevy_polyline::*;

use crate::{PhysicsTime, prelude::{Body, ContactBroad, Contact, ConstraintPenetration}, manifold::Manifold};

#[derive(Inspectable,Default, Debug, Copy, Clone)]
pub struct PhysicsReport {
    time:  f32,
    #[inspectable()]
    bodies:    usize,
    manifolds: usize,
    collision_pairs:   usize,
    contacts:  usize,
    constraint: usize,
}

pub fn report_system(
    pt: Res<PhysicsTime>,
    bodies: Query<(&Body, &Transform)>,
    mut collision_pairs: EventReader<ContactBroad>,
    mut contacts: EventReader<Contact>,
    manifolds: Query<&Manifold>,
    constraint_penetrations: Query<&ConstraintPenetration>,
    mut report: ResMut<PhysicsReport>,
) {
        report.time = pt.time;
        report.bodies =  bodies.iter().count();
        report.manifolds = manifolds.iter().count();
        report.collision_pairs = collision_pairs.iter().count();
        report.contacts = contacts.iter().count();
        report.constraint = constraint_penetrations.iter().count();

}

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

        }),
        material: polyline_materials.add(PolylineMaterial {
            width: 3.0,
            color: Color::RED,
            perspective: false,

        }),
        ..Default::default()
    });
}