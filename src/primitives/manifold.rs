use crate::{
    constraints::{ConstraintPenetration, ConstraintConfig},

};
use bevy::prelude::*;

use super::{Contact, Body};

const MAX_CONTACTS: usize = 4;

#[derive(Component)]
pub struct Manifold {
    pub handle_a: Entity,
    pub handle_b: Entity,
    pub contact_contraints: Vec<ManifoldConstraint>,
}

pub struct ManifoldConstraint {
    pub contact: Contact,
    pub constraint_entity: Entity,
}

impl Manifold {
    pub fn new(handle_a: Entity, handle_b: Entity) -> Self {
        Self {
            contact_contraints: Vec::with_capacity(MAX_CONTACTS),
            handle_a,
            handle_b,
        }
    }

    pub fn add_contact(
        &mut self,
        commands: &mut Commands,
        body_a: &Body,
        transform_a: &GlobalTransform,
        body_b: &Body,
        transform_b: &GlobalTransform,
        bodies: &Query<(&Body, &GlobalTransform)>,
        contact: Contact,
    ) {

        // check existing contacts
        for cc in &self.contact_contraints {
            // if this contact is close to another contact then keep the old contact
            let old_a = body_a.local_to_world(transform_a, cc.contact.local_point_a);
            let old_b = body_b.local_to_world(transform_b, cc.contact.local_point_b);

            // TODO: Revisit this
            unsafe {
                let (new_body_a, new_trans_a) = bodies.get_unchecked(contact.entity_a).unwrap();
                let (new_body_b, new_trans_b) = bodies.get_unchecked(contact.entity_b).unwrap();

                let new_a = new_body_a.local_to_world(new_trans_a, contact.local_point_a);
                let new_b = new_body_b.local_to_world(new_trans_b, contact.local_point_b);

                let aa = new_a - old_a;
                let bb = new_b - old_b;

                const DISTANCE_THRESHOLD: f32 = 0.02;
                const DISTANCE_THRESHOLD_SQ: f32 = DISTANCE_THRESHOLD * DISTANCE_THRESHOLD;
                if aa.length_squared() < DISTANCE_THRESHOLD_SQ {
                    return;
                }
                if bb.length_squared() < DISTANCE_THRESHOLD_SQ {
                    return;
                }
            }
        }

        let mut new_slot = self.contact_contraints.len();
        if self.contact_contraints.len() >= MAX_CONTACTS {
            // find the avg point
            let mut avg = Vec3::ZERO;
            for cc in &self.contact_contraints {
                avg += cc.contact.local_point_a;
            }
            avg += contact.local_point_a;
            avg *= 1.0 / (self.contact_contraints.len() + 1) as f32;

            // find farthest contact
            let mut min_dist = (avg - contact.local_point_a).length_squared();
            let mut new_idx = None;
            for i in 0..MAX_CONTACTS {
                let dist2 =
                    (avg - self.contact_contraints[i].contact.local_point_a).length_squared();

                if dist2 < min_dist {
                    min_dist = dist2;
                    new_idx = Some(i);
                }
            }

            if let Some(new_idx) = new_idx {
                new_slot = new_idx;
            } else {
                // new contact is the farthest away, exit
                return;
            }
        }

        // build contraint
        let mut normal = transform_a.rotation.inverse() * -contact.normal;
        normal = normal.normalize();
        let constraint = ConstraintPenetration::new(
            ConstraintConfig {
                handle_a: contact.entity_a,
                handle_b: contact.entity_b,
                anchor_a: contact.local_point_a,
                anchor_b: contact.local_point_b,
                ..ConstraintConfig::default()
            },
            normal,
        );
        let constraint_entity = commands.spawn().insert(constraint).id();

        // add or replace contact
        if new_slot == self.contact_contraints.len() {
            self.contact_contraints.push(ManifoldConstraint {
                contact,
                constraint_entity,
            });
        } else {
            self.contact_contraints[new_slot] = ManifoldConstraint {
                contact,
                constraint_entity,
            };
        }
    }
}

pub fn manifold_remove_expired_system(
    mut commands: Commands,
    bodies: Query<(&Body, &GlobalTransform)>,
    mut manifolds: Query<(Entity, &mut Manifold)>,
    constraints: Query<&ConstraintPenetration>,
) {
    for (e, mut manifold) in manifolds.iter_mut() {
        unsafe {
            let a = bodies.get_unchecked(manifold.handle_a);
            let b = bodies.get_unchecked(manifold.handle_b);

            if a.is_err() || b.is_err() {
                warn!(
                    "Manifold invalid entity handles, handle_a: {:?}, handle_b: {:?}",
                    manifold.handle_a, manifold.handle_b
                );
                for cc in &manifold.contact_contraints {
                    commands.entity(cc.constraint_entity).despawn();
                }
                commands.entity(e).despawn();
                continue;
            }

            let (body_a, trans_a) = a.unwrap();
            let (body_b, trans_b) = b.unwrap();

            if manifold.contact_contraints.len() >= MAX_CONTACTS {
                // find contacts that have drifted too far
                manifold.contact_contraints.retain(|cc| {
                    // get the tangential distance of the point on a and the point on b
                    let pos_a = body_a.local_to_world(trans_a, cc.contact.local_point_a);
                    let pos_b = body_b.local_to_world(trans_b, cc.contact.local_point_b);

                    let contraint = constraints.get(cc.constraint_entity).unwrap();
                    let normal = trans_a.rotation * contraint.normal();

                    // calculate the tangential separation and penetration depth
                    let ab = pos_b - pos_a;
                    let penetration_depth = normal.dot(ab);
                    let ab_normal = normal * penetration_depth;
                    let ab_tangent = ab - ab_normal;

                    // if the tangential displacement is less than a specific threshold, it's okay to keep
                    // it.
                    const DISTANCE_THRESHOLD: f32 = 0.02;
                    if ab_tangent.length_squared() < DISTANCE_THRESHOLD * DISTANCE_THRESHOLD
                        && penetration_depth <= 0.0
                    {
                        return true;
                    }

                    // this contact has moved beyond its threshold and should be removed
                    warn!("Remove Constraint: {:?}", cc.constraint_entity);
                    commands.entity(cc.constraint_entity).despawn();
                    false
                });
            }
        }

        // Clean up self if empty
        // if manifold.contact_contraints.is_empty() {
        //     warn!("Manifold Empty: {:?}", e);
        //     commands.entity(e).despawn();
        // }
    }
}
