mod gjk;
mod sphere;
mod aabb;

pub use gjk::*;
pub use sphere::*;
pub use aabb::*;

use bevy::prelude::*;

use crate::primitives::{Body, Contact};

#[cfg(feature = "disabled")]
pub fn intersect_static(
    entity_a: Entity,
    body_a: &Body,
    transform_a: &Transform,
    entity_b: Entity,
    body_b: &Body,
    transform_b: &Transform,
) -> (Contact, bool) {
    match (body_a.collider.shape, body_b.collider.shape) {
        (ShapeType::Sphere { radius: radius_a }, ShapeType::Sphere { radius: radius_b }) => {
            let pos_a = transform_a.translation;
            let pos_b = transform_b.translation;

            if let Some((world_point_a, world_point_b)) =
                sphere_sphere_static(radius_a, radius_b, pos_a, pos_b)
            {
                (
                    Contact {
                        entity_a,
                        entity_b,
                        world_point_a,
                        world_point_b,
                        local_point_a: body_a.world_to_local(transform_a, world_point_a),
                        local_point_b: body_b.world_to_local(transform_b, world_point_b),
                        normal: (pos_a - pos_b).normalize(),
                        separation_dist: (world_point_a - world_point_b).length(),
                        time_of_impact: 0.0,
                    },
                    true,
                )
            } else {
                (
                    Contact {
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
                    Contact {
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
                let (world_point_a, world_point_b) =
                    gjk_closest_points(body_a, transform_a, body_b, transform_b);
                (
                    Contact {
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

#[cfg(feature = "disabled")]
pub fn intersect_dynamic(
    entity_a: Entity,
    body_a: &Body,
    transform_a: &Transform,
    entity_b: Entity,
    body_b: &Body,
    transform_b: &Transform,
    dt: f32,
) -> Option<Contact> {
    // skip body pairs with infinite mass
    if body_a.has_infinite_mass() && body_b.has_infinite_mass() {
        return None;
    }

    // test for intersections, fire contact event if necessary
    let shapes = (body_a.collider.shape, body_b.collider.shape);
    match shapes {
        (ShapeType::Sphere { radius: radius_a }, ShapeType::Sphere { radius: radius_b }) => {
            if let Some((world_point_a, world_point_b, time_of_impact)) = sphere::sphere_sphere_dynamic(
                radius_a,
                radius_b,
                transform_a.translation,
                transform_b.translation,
                body_a.linear_velocity,
                body_b.linear_velocity,
                dt,
            ) {
                // simulate moving forward to get local space collision points
                let (pos_a, local_point_a) =
                    body_a.local_collision_point(transform_a, time_of_impact, world_point_a);
                let (pos_b, local_point_b) =
                    body_b.local_collision_point(transform_b, time_of_impact, world_point_b);

                let normal = (pos_a - pos_b).normalize();

                // calculate the separation distance
                let ab = transform_a.translation - transform_b.translation;
                let separation_dist = ab.length() - (radius_a + radius_b);

                Some(Contact {
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
        // TODO: implement support for other shapes
        (_, _) => {
            // use GJK to perform conservative advancement
            conservative_advance(
                entity_a,
                body_a,
                transform_a,
                entity_b,
                body_b,
                transform_b,
                dt,
            )
        }
    }
}

#[cfg(feature = "disabled")]
fn conservative_advance(
    entity_a: Entity,
    body_a: &Body,
    transform_a: &Transform,
    entity_b: Entity,
    body_b: &Body,
    transform_b: &Transform,
    mut dt: f32,
) -> Option<Contact> {
    let mut toi = 0.0;
    let mut num_iters = 0;

    // TODO: Yes copies here are bad, but the old way required mutable access,
    // need to figure better way to simulate update
    let mut tmp_body_a = body_a.to_owned();
    let mut tmp_body_b = body_b.to_owned();

    let mut tmp_trans_a = transform_a.to_owned();
    let mut tmp_trans_b = transform_b.to_owned();

    // advance the positions of the bodies until they touch or there's not time left
    while dt > 0.0 {
        // check for intersection
        let (mut contact, did_intersect) =
            intersect_static(entity_a, body_a, transform_a, entity_b, body_b, transform_b);
        if did_intersect {
            contact.time_of_impact = toi;
            tmp_body_a.update(&mut tmp_trans_a, -toi);
            tmp_body_b.update(&mut tmp_trans_b, -toi);
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
        tmp_body_a.update(&mut tmp_trans_a, time_to_go);
        tmp_body_b.update(&mut tmp_trans_a, time_to_go);
    }

    None
}

