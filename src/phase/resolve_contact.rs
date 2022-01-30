use crate::{
    constraints::{ConstraintConfig, ConstraintPenetration},
    primitives::*,
    PhysicsTime,
};

use bevy::prelude::*;

pub fn resolve_contact_system(
    pt: Res<PhysicsTime>,
    mut query: Query<(&mut Body, &mut GlobalTransform)>,
    mut contacts: EventReader<Contact>,
) {
    #[cfg(feature = "static")]
    {
        for contact in contacts.iter() {
            unsafe {
                let a = query.get_unchecked(contact.entity_a);
                let b = query.get_unchecked(contact.entity_b);
                if a.is_err() || b.is_err() {
                    continue;
                }

                let (mut body_a, mut transform_a) = a.unwrap();
                let (mut body_b, mut transform_b) = b.unwrap();

                resolve_contact(
                    contact,
                    &mut body_a,
                    &mut transform_a,
                    &mut body_b,
                    &mut transform_b,
                );
            }
        }

        // Apply ballistic impulses
        for (mut body, mut transform) in query.iter_mut() {
            body.update(&mut transform, pt.time)
        }
    }

    #[cfg(feature = "dynamic")]
    {
        // sort the times of impact from earliest to latest
        let mut list = contacts.iter().collect::<Vec<_>>();

        info!("contats: {}", list.len());
        list.sort_unstable_by(|a, b| a.time_of_impact.partial_cmp(&b.time_of_impact).unwrap());

        // Apply Ballistic impulses
        let mut accumulated_time = 0.0;
        for contact in list.iter() {
            let contact_time = contact.time_of_impact - accumulated_time;

            // position update
            for (mut body, mut transform) in query.iter_mut() {
                body.update(&mut transform, contact_time);
            }

            unsafe {
                let (mut body_a, mut transform_a) = query.get_unchecked(contact.entity_a).unwrap();
                let (mut body_b, mut transform_b) = query.get_unchecked(contact.entity_b).unwrap();
                resolve_contact(
                    contact,
                    &mut body_a,
                    &mut transform_a,
                    &mut body_b,
                    &mut transform_b,
                );
            }
            accumulated_time += contact_time;
        }

        //update positions for the rest of this frame's time
        let time_remaining = pt.time - accumulated_time;
        if time_remaining > 0.0 {
            for (mut body, mut transform) in query.iter_mut() {
                body.update(&mut transform, time_remaining)
            }
        }
    }
}

fn resolve_contact(
    contact: &Contact,
    body_a: &mut Body,
    transform_a: &mut GlobalTransform,
    body_b: &mut Body,
    transform_b: &mut GlobalTransform,
) {
    let elasticity = body_a.elasticity * body_b.elasticity;
    let total_inv_mass = body_a.inv_mass + body_b.inv_mass;

    let inv_inertia_world_a = body_a.inv_inertia_tensor_world(transform_a);
    let inv_inertia_world_b = body_b.inv_inertia_tensor_world(transform_b);

    let ra = contact.world_point_a - body_a.centre_of_mass_world(transform_a);
    let rb = contact.world_point_b - body_b.centre_of_mass_world(transform_b);

    let angular_j_a = (inv_inertia_world_a * ra.cross(contact.normal)).cross(ra);
    let angular_j_b = (inv_inertia_world_b * rb.cross(contact.normal)).cross(rb);
    let angular_factor = (angular_j_a + angular_j_b).dot(contact.normal);

    // Get the world space velocity of the motion and rotation
    let vel_a = body_a.linear_velocity + body_a.angular_velocity.cross(ra);
    let vel_b = body_b.linear_velocity + body_b.angular_velocity.cross(rb);

    // Calculate the collion impulse
    let vab = vel_a - vel_b;
    let impluse_j =
        -(1.0 + elasticity) * vab.dot(contact.normal) / (total_inv_mass + angular_factor);
    let impluse_vec_j = contact.normal * impluse_j;

    body_a.apply_impulse(contact.world_point_a, impluse_vec_j, transform_a);
    body_b.apply_impulse(contact.world_point_b, -impluse_vec_j, transform_b);

    //
    // Calculate the friction impulse
    //
    let friction = body_a.friction * body_b.friction;

    // Find the normal direction of the velocity with respoect to the normal of the collison
    let velocity_normal = contact.normal * contact.normal.dot(vab);
    let velocity_tangent = vab - velocity_normal;

    // Get the tangent velocities relative to the other body
    let relative_velocity_tangent = velocity_tangent.normalize();

    let inertia_a = (inv_inertia_world_a * ra.cross(relative_velocity_tangent)).cross(ra);
    let inertia_b = (inv_inertia_world_b * rb.cross(relative_velocity_tangent)).cross(rb);
    let inv_inertia = (inertia_a + inertia_b).dot(relative_velocity_tangent);

    // calculat the tangential impluse for friction
    let reduced_mass = 1.0 / (total_inv_mass + inv_inertia);
    let impluse_friction = velocity_tangent * (reduced_mass * friction);

    // TODO: Book didnt have this if check, but I was getitng velocity_tangent of zero leading to
    // a Vec3 Nan when normalized if perfectly lined up on ground
    if !impluse_friction.is_nan() {
        // apply kinetic friction
        body_a.apply_impulse(contact.world_point_a, -impluse_friction, transform_a);
        body_b.apply_impulse(contact.world_point_b, impluse_friction, transform_b);
    }

    //
    // lets move the bodies part if intersecting
    //
    if contact.time_of_impact == 0.0 {
        // Let's also move our colliding objects to just outside of each other
        let a_move_weight = body_a.inv_mass / total_inv_mass;
        let b_move_weight = body_b.inv_mass / total_inv_mass;

        let direction = contact.world_point_b - contact.world_point_a;

        transform_a.translation += direction * a_move_weight;
        transform_b.translation -= direction * b_move_weight;
    }
}

const MAX_CONTACTS: usize = 4;
pub fn add_manifold_contact_system(
    commands: &mut Commands,
    mut contacts: EventReader<ManifoldContactEvent>,
    query: Query<(&mut Body, &mut GlobalTransform)>,
    mut manifolds: Query<&mut Manifold>,
) {
    for contact in contacts.iter() {
        let mut contact = contact.0.to_owned();

        unsafe {
            let (body_a, transform_a) = query.get_unchecked(contact.entity_a).unwrap();
            let (body_b, transform_b) = query.get_unchecked(contact.entity_b).unwrap();

            let mut manifold = manifolds
                .iter_mut()
                .find(|m| {
                    let has_a = m.handle_a == contact.entity_a || m.handle_b == contact.entity_a;
                    let has_b = m.handle_a == contact.entity_b || m.handle_b == contact.entity_b;
                    if has_a && has_b {
                        return true;
                    }
                    false
                })
                .unwrap();

            // make sure the contact's body_a and body_b are of the correct order
            if contact.entity_a != manifold.handle_a || contact.entity_b != manifold.handle_b {
                std::mem::swap(&mut contact.local_point_a, &mut contact.local_point_b);
                std::mem::swap(&mut contact.world_point_a, &mut contact.world_point_b);
                std::mem::swap(&mut contact.entity_a, &mut contact.entity_b);
            }

            // check existing contacts

            let new_a = body_a.local_to_world(&transform_a, contact.local_point_a);
            let new_b = body_b.local_to_world(&transform_b, contact.local_point_b);

            for cc in &manifold.contact_contraints {
                // if this contact is close to another contact then keep the old contact
                let old_a = body_a.local_to_world(&transform_a, cc.contact.local_point_a);
                let old_b = body_b.local_to_world(&transform_b, cc.contact.local_point_b);

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
                    let dist2 = (avg - manifold.contact_contraints[i].contact.local_point_a)
                        .length_squared();

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
                manifold.contact_contraints.push(ManifoldConstraint {
                    contact,
                    constraint_entity,
                });
            } else {
                manifold.contact_contraints[new_slot] = ManifoldConstraint {
                    contact,
                    constraint_entity,
                };
            }
        }
    }
}
