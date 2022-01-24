use crate::PhysicsTime;
use crate::{body::Body, contact::Contact};

use bevy::prelude::*;

pub fn resolve_contact_system(
    pt: Res<PhysicsTime>,
    mut query: Query<(Entity, &mut Body, &mut Transform)>,
    mut contacts: EventReader<Contact>,
) {
      for contact in contacts.iter() {
        unsafe {
            let (_, mut body_a, mut transform_a) = query.get_unchecked(contact.entity_a).unwrap();
            let (_, mut body_b, mut transform_b) = query.get_unchecked(contact.entity_b).unwrap();
            resolve_contact(
                contact,
                &mut body_a,
                &mut transform_a,
                &mut body_b,
                &mut transform_b,
            );
        }
    }

    //position update
    for (_, mut body, mut transform) in query.iter_mut() {
        body.update(&mut transform, pt.time)
    }


    // // Apply Ballistic impulses
    // let mut accumulated_time = 0.0;
    // for contact in contacts.iter() {
    //     let contact_time = contact.time_of_impact - accumulated_time;

    //     //position update
    //     for (_, mut body, mut transform) in query.iter_mut() {
    //         body.update(&mut transform, contact_time)
    //     }

    //     unsafe {
    //         let (_, mut body_a, mut transform_a) = query.get_unchecked(contact.entity_a).unwrap();
    //         let (_, mut body_b, mut transform_b) = query.get_unchecked(contact.entity_b).unwrap();
    //         resolve_contact(
    //             contact,
    //             &mut body_a,
    //             &mut transform_a,
    //             &mut body_b,
    //             &mut transform_b,
    //         );
    //     }
    //     accumulated_time += contact_time;
    // }

    // //update positions for the rest of this frame's time
    // let time_remaining = pt.time - accumulated_time;
    // if time_remaining > 0.0 {
    //     for (_, mut body, mut transform) in query.iter_mut() {
    //         body.update(&mut transform, time_remaining)
    //     }
    // }
}


fn resolve_contact(
    contact: &Contact,
    body_a: &mut Body,
    transform_a: &mut Transform,
    body_b: &mut Body,
    transform_b: &mut Transform,
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
