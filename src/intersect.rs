use bevy::prelude::*;

use crate::prelude::{Body, ContactEvent, ShapeType};
use crate::gjk::{gjk_does_intersect, gjk_closest_points};

fn intersect_static(
    entity_a: Entity,
    body_a: &mut Body,
    transform_a: &mut Transform,
    entity_b: Entity,
    body_b: &mut Body,
    transform_b: &mut Transform,
) -> (ContactEvent, bool) {
    match (body_a.collider.shape, body_b.collider.shape) {
        (ShapeType::Sphere { radius: radius_a}, ShapeType::Sphere {radius: radius_b} ) => {
            let pos_a = transform_a.translation;
            let pos_b = transform_b.translation;

            if let Some((world_point_a, world_point_b)) =
                sphere_sphere_static(radius_a, radius_b, pos_a, pos_b)
            {
                (
                    ContactEvent {
                        entity_a,
                        entity_b,
                        world_point_a,
                        world_point_b,
                        local_point_a: body_a.world_to_local(&transform_a, world_point_a),
                        local_point_b: body_b.world_to_local(&transform_b, world_point_b),
                        normal: (pos_a - pos_b).normalize(),
                        separation_dist: (world_point_a - world_point_b).length(),
                        time_of_impact: 0.0,
                    },
                    true,
                )
            } else {
                (
                    ContactEvent {
                        entity_a,
                        entity_b,
                        world_point_a: Vec3::ZERO,
                        world_point_b: Vec3::ZERO,
                        local_point_a: Vec3::ZERO,
                        local_point_b: Vec3::ZERO,
                        normal: Vec3::X,
                        separation_dist: 0.0,
                        time_of_impact: 0.0,
                    },
                    false,
                )
            }
        }
        (_, _) => {
            const BIAS: f32 = 0.001;
            if let Some((mut world_point_a, mut world_point_b)) =
                gjk_does_intersect(body_a, transform_a, body_b, transform_b, BIAS)
            {
                let normal = (world_point_b - world_point_a).normalize_or_zero();
                world_point_a -= normal * BIAS;
                world_point_b += normal * BIAS;
                (
                    ContactEvent {
                        entity_a,
                        entity_b,
                        world_point_a,
                        world_point_b,
                        local_point_a: body_a.world_to_local(transform_a, world_point_a),
                        local_point_b: body_b.world_to_local(transform_b, world_point_b),
                        normal,
                        separation_dist: -(world_point_a - world_point_b).length(),
                        time_of_impact: 0.0,
                    },
                    true,
                )
            } else {
                let (world_point_a, world_point_b) = gjk_closest_points(body_a, transform_a, body_b, transform_b);
                (
                    ContactEvent {
                        entity_a,
                        entity_b,
                        world_point_a,
                        world_point_b,
                        local_point_a: body_a.world_to_local(transform_a, world_point_a),
                        local_point_b: body_b.world_to_local(transform_b, world_point_b),
                        normal: Vec3::ZERO,
                        separation_dist: (world_point_a - world_point_b).length(),
                        time_of_impact: 0.0,
                    },
                    false,
                )
            }
        }
    }
}

pub fn intersect_dynamic(
    entity_a: Entity,
    body_a: &mut Body,
    transform_a: &mut Transform,
    entity_b: Entity,
    body_b: &mut Body,
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
        (ShapeType::Sphere { radius: radius_a} , ShapeType::Sphere { radius: radius_b}) => {
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
                    entity_a,
                    entity_b,
                })
            } else {
                None
            }
        }
        (_, _) => {
            // use GJK to perform conservative advancement
            conservative_advance(entity_a, body_a, transform_a,  entity_b, body_b, transform_b, dt)

        },
    }
}


fn conservative_advance(
    entity_a: Entity,
    body_a: &mut Body,
    transform_a: &mut Transform,
    entity_b: Entity,
    body_b: &mut Body,
    transform_b: &mut Transform,
    mut dt: f32,
) -> Option<ContactEvent> {
    let mut toi = 0.0;
    let mut num_iters = 0;

    // advance the positions of the bodies until they touch or there's not time left
    while dt > 0.0 {
        // check for intersection
        let (mut contact, did_intersect) = intersect_static(entity_a, body_a, transform_a, entity_b, body_b, transform_b);
        if did_intersect {
            contact.time_of_impact = toi;
            body_a.update(transform_a, -toi);
            body_b.update(transform_b, -toi);
            return Some(contact);
        }

        num_iters += 1;
        if num_iters > 10 {
            break;
        }

        // get the vector from the closest point on A to the closest point on B
        let ab = (contact.world_point_b - contact.world_point_a).normalize_or_zero();

        // project the relative velocity onto the ray of shortest distance
        let relative_velocity = body_a.linear_velocity - body_b.linear_velocity;
        let mut ortho_speed = relative_velocity.dot(ab);

        // add to the ortho_speed the maximum angular speeds of the relative shapes
        let angular_speed_a = body_a
            .collider
            .fastest_linear_speed(body_a.angular_velocity, ab);
        let angular_speed_b = body_b
            .collider
            .fastest_linear_speed(body_b.angular_velocity, -ab);
        ortho_speed += angular_speed_a + angular_speed_b;

        if ortho_speed <= 0.0 {
            break;
        }

        let time_to_go = contact.separation_dist / ortho_speed;
        if time_to_go > dt {
            break;
        }

        dt -= time_to_go;
        toi += time_to_go;
        body_a.update(transform_a,time_to_go);
        body_b.update(transform_b,time_to_go);
    }

    // unwind the clock
    body_a.update(transform_a,-toi);
    body_b.update(transform_b,-toi);

    None
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

pub fn sphere_sphere_static(
    radius_a: f32,
    radius_b: f32,
    pos_a: Vec3,
    pos_b: Vec3,
) -> Option<(Vec3, Vec3)> {
    let ab = pos_b - pos_a;
    let radius_ab = radius_a + radius_b;
    let length_squared = ab.length_squared();
    if length_squared < radius_ab * radius_ab {
        let norm = ab.normalize_or_zero();
        let pt_on_a = pos_a + norm * radius_a;
        let pt_on_b = pos_b - norm * radius_b;
        Some((pt_on_a, pt_on_b))
    } else {
        None
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

