use bevy::prelude::*;

use crate::prelude::{RigidBody, ContactEvent, ShapeType};

pub fn intersect_dynamic(
    a: Entity,
    body_a: &mut RigidBody,
    transform_a: &mut Transform,
    b: Entity,
    body_b: &mut RigidBody,
    transform_b: &mut Transform,
    dt: f32,
) -> Option<ContactEvent> {
    // skip body pairs with infinite mass
    if body_a.inv_mass == 0.0 && body_b.inv_mass == 0.0 {
        return None;
    }

    // test for intersections, fire contact event if necessary
    let shapes = (body_a.collider.shape, body_b.collider.shape);
    match shapes {
        (ShapeType::Sphere, ShapeType::Sphere) => {
            let radius_a = body_a.collider.bounds.maxs.x;
            let radius_b = body_b.collider.bounds.maxs.x;
            if let Some((world_point_a, world_point_b, time_of_impact)) = sphere_sphere_dynamic(
                radius_a,
                radius_b,
                transform_a.translation,
                transform_b.translation,
                body_a.linear_velocity,
                body_b.linear_velocity,
                dt,
            ) {
                // step bodies forward to get local space collision points
                body_a.update(transform_a, time_of_impact);
                body_b.update(transform_b, time_of_impact);

                // convert world space contacts to local space
                let local_point_a = body_a.world_to_local(transform_a, world_point_a);
                let local_point_b = body_b.world_to_local(transform_b, world_point_b);

                let normal = (transform_a.translation - transform_b.translation).normalize();

                // unwind time step
                body_a.update(transform_a, -time_of_impact);
                body_b.update(transform_b, -time_of_impact);

                // calculate the separation distance
                let ab = transform_a.translation - transform_b.translation;
                let separation_dist = ab.length() - (radius_a + radius_b);

                Some(ContactEvent {
                    world_point_a,
                    world_point_b,
                    local_point_a,
                    local_point_b,
                    normal,
                    separation_dist,
                    time_of_impact,
                    entity_a: a,
                    entity_b: b,
                })
            } else {
                None
            }
        }
        (ShapeType::Sphere, ShapeType::Box) => todo!(),
        (ShapeType::Sphere, ShapeType::Convex) => todo!(),
        (ShapeType::Box, ShapeType::Sphere) => todo!(),
        (ShapeType::Box, ShapeType::Box) => todo!(),
        (ShapeType::Box, ShapeType::Convex) => todo!(),
        (ShapeType::Convex, ShapeType::Sphere) => todo!(),
        (ShapeType::Convex, ShapeType::Box) => todo!(),
        (ShapeType::Convex, ShapeType::Convex) => todo!(),
    }
}

pub fn ray_sphere_intersect(
    ray_start: Vec3,
    ray_direction: Vec3,
    sphere_center: Vec3,
    sphere_radius: f32,
) -> Option<(f32, f32)> {
    let m = sphere_center - ray_start;
    let a = ray_direction.dot(ray_direction);
    let b = m.dot(ray_direction);
    let c = m.dot(m) - sphere_radius * sphere_radius;

    let delta = b * b - a * c;

    if delta < 0.0 {
        None
    } else {
        let inv_a = 1.0 / a;
        let delta_root = delta.sqrt();
        let t1 = inv_a * (b - delta_root);
        let t2 = inv_a * (b + delta_root);
        Some((t1, t2))
    }
}


fn sphere_sphere_dynamic(
    radius_a: f32,
    radius_b: f32,
    pos_a: Vec3,
    pos_b: Vec3,
    vel_a: Vec3,
    vel_b: Vec3,
    dt: f32,
) -> Option<(Vec3, Vec3, f32)> {
    let relative_velocity = vel_a - vel_b;

    let start_pt_a = pos_a;
    let end_pt_a = pos_a + relative_velocity * dt;
    let ray_dir = end_pt_a - start_pt_a;

    let mut t0 = 0.0;
    let mut t1 = 0.0;

    const EPSILON: f32 = 0.001;
    const EPSILON_SQ: f32 = EPSILON * EPSILON;
    if ray_dir.length_squared() < EPSILON_SQ {
        // ray is too short, just check if intersecting
        let ab = pos_b - pos_a;
        let radius = radius_a + radius_b + EPSILON;
        if ab.length_squared() > radius * radius {
            return None;
        }
    } else if let Some(toi) = ray_sphere_intersect(pos_a, ray_dir, pos_b, radius_a + radius_b) {
        t0 = toi.0;
        t1 = toi.1;
    } else {
        return None;
    }

    // Change from [0,1] range to [0,dt] range
    t0 *= dt;
    t1 *= dt;

    // If the collision is only in the past, then there's not future collision this frame
    if t1 < 0.0 {
        return None;
    }

    // Get the earliest positive time of impact
    let toi = if t0 < 0.0 { 0.0 } else { t0 };

    // If the earliest collision is too far in the future, then there's no collision this frame
    if toi > dt {
        return None;
    }

    // get the points on the respective points of collision
    let new_pos_a = pos_a + vel_a * toi;
    let new_pos_b = pos_b + vel_b * toi;
    let ab = (new_pos_b - new_pos_a).normalize();

    let pt_on_a = new_pos_a + ab * radius_a;
    let pt_on_b = new_pos_b - ab * radius_b;

    Some((pt_on_a, pt_on_b, toi))
}

