use bevy::prelude::*;

use crate::{
    body::Body,
    intersect::intersect_dynamic,
    prelude::{ConstraintConfig, ConstraintPenetration, Contact},
    CollisionPairVec, ContactVec, PhysicsTime,
};

use super::{Manifold, ContactConstraint};
const MAX_CONTACTS: usize = 4;

// Narrowphase (perform actual collision detection)
pub fn narrowphase_system(
    mut commands: Commands,
    pt: Res<PhysicsTime>,
    bodies: Query<(&Body, &Transform)>,
    collision_pairs: Res<CollisionPairVec>,
    mut contacts: ResMut<ContactVec>,
    mut manifolds: Query<&mut Manifold>,
) {
    contacts.0.clear();

    // test possable contacts collisions
    for pair in collision_pairs.0.iter() {
        // SAFETY: There is no way for a and b to the same entity
        // see https://github.com/bevyengine/bevy/issues/2042
        unsafe {
            let (body_a, transform_a) = bodies.get_unchecked(pair.a).unwrap();
            let (body_b, transform_b) = bodies.get_unchecked(pair.b).unwrap();

            // skip body pairs with infinite mass
            if body_a.has_infinite_mass() && body_b.has_infinite_mass() {
                continue;
            }

            // check for intersection, and make sure entity a is less b, will save checks later
            let contact_result = if pair.a <= pair.b {
                intersect_dynamic(
                    pair.a,
                    body_a,
                    transform_a,
                    pair.b,
                    body_b,
                    transform_b,
                    pt.time,
                )
            } else {
                intersect_dynamic(
                    pair.b,
                    body_b,
                    transform_b,
                    pair.a,
                    body_a,
                    transform_a,
                    pt.time,
                )
            };

            if let Some(contact) = contact_result {

                info!("Contact found between {:?} and {:?}", pair.a, pair.b);
                
                if contact.time_of_impact == 0.0 {
                    // static contact (already touching)

                    // try to find the previously existing manifold for contacts between two bodies
                    let mut found = None;
                    for manifold in &mut manifolds.iter_mut() {
                        // we are assuming handles in both manifold and contact match
                        if manifold.handle_a == contact.entity_a
                            && manifold.handle_b == contact.entity_b
                        {
                            found = Some(manifold);
                            break;
                        }
                    }


                    if let Some(mut manifold) = found {
                        //existing manifold, add the contact to it
                        add_manifold_contact(
                            &mut commands,
                            &mut manifold,
                            body_a,
                            transform_a,
                            body_b,
                            transform_b,
                            &bodies,
                            contact,
                        );
                    } else {
                        // we don't have an existing manifold, create a new one
                        let mut new_manifold = Manifold::new(contact.entity_a, contact.entity_b);
                        add_manifold_contact(
                            &mut commands,
                            &mut new_manifold,
                            body_a,
                            transform_a,
                            body_b,
                            transform_b,
                            &bodies,
                            contact,
                        );
                        commands
                            .spawn()
                            .insert(new_manifold)
                            .insert(Name::new("Manifold"));
                    }
                } else {
                    // ballistic contact
                    contacts.0.push(contact)
                }
            }
        }
    }

    // sort the times of impact from earliest to latest
    contacts.0.sort_unstable_by(|a, b| {
        if a.time_of_impact < b.time_of_impact {
            std::cmp::Ordering::Less
        } else if a.time_of_impact == b.time_of_impact {
            std::cmp::Ordering::Equal
        } else {
            std::cmp::Ordering::Greater
        }
    });
}

fn add_manifold_contact(
    commands: &mut Commands,
    manifold: &mut Manifold,
    body_a: &Body,
    transform_a: &Transform,
    body_b: &Body,
    transform_b: &Transform,
    bodies: &Query<(&Body, &Transform)>,
    contact: Contact,
) {
    // check existing contacts
    for cc in &manifold.contact_contraints {

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

    let mut new_slot = manifold.contact_contraints.len();
    if manifold.contact_contraints.len() >= MAX_CONTACTS {
        // find the avg point
        let mut avg = Vec3::ZERO;
        for cc in &manifold.contact_contraints {
            avg += cc.contact.local_point_a;
        }
        avg += contact.local_point_a;
        avg *= 1.0 / (manifold.contact_contraints.len() + 1) as f32;

        // find farthest contact
        let mut min_dist = (avg - contact.local_point_a).length_squared();
        let mut new_idx = None;
        for i in 0..MAX_CONTACTS {
            let dist2 = (avg - manifold.contact_contraints[i].contact.local_point_a).length_squared();

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
    if new_slot == manifold.contact_contraints.len() {
        manifold.contact_contraints.push(ContactConstraint {
            contact,
            constraint_entity,
        });
    } else {
        manifold.contact_contraints[new_slot] =  ContactConstraint {
            contact,
            constraint_entity,
        };
    }
}
