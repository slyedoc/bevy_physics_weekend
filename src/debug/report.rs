use bevy::prelude::*;
use bevy_inspector_egui::Inspectable;

use crate::{constraints::ConstraintPenetration, primitives::*, PhysicsTime};

#[derive(Inspectable, Default, Debug, Copy, Clone)]
// TODO: Make this disable so user knows they can't change anything
pub struct PhysicsReport {
    time: f32,
    #[inspectable()]
    bodies: usize,
    manifolds: usize,
    collision_pairs: usize,
    contacts: usize,
    constraint: usize,
}

pub fn report_system(
    pt: Res<PhysicsTime>,
    bodies: Query<(&Body, &Transform)>,
    mut collision_pairs: EventReader<BroadContact>,
    mut contacts: EventReader<Contact>,
    manifolds: Query<&Manifold>,
    constraint_penetrations: Query<&ConstraintPenetration>,
    mut report: ResMut<PhysicsReport>,
) {
    report.time = pt.time;
    report.bodies = bodies.iter().count();
    report.manifolds = manifolds.iter().count();
    report.collision_pairs = collision_pairs.iter().count();
    report.contacts = contacts.iter().count();
    report.constraint = constraint_penetrations.iter().count();
}
