use bevy::{prelude::*, render::primitives::Aabb};

use super::CollisionPair;
use crate::{body::Body, CollisionPairVec, PhysicsTime};
#[derive(Copy, Clone, Debug)]
pub struct PsuedoBody {
    entity: Entity,
    value: f32,
    is_min: bool,
}

// Broadphase (build potential collision pairs)
// not currently dynamic, wanted to test using the render Aabb
pub fn broadphase_system(
    query: Query<(Entity, &Transform, &Body, &Aabb)>,
    pt: Res<PhysicsTime>,
    mut collision_pairs: ResMut<CollisionPairVec>,
) {
    collision_pairs.0.clear();

    let mut iter = query.iter_combinations();
    while let Some([(e1, trans_a, body_a, aabb_a), (e2, trans_b, body_b, aabb_b)]) =
        iter.fetch_next()
    {
        let mut aabb_a = aabb_a.clone();
        aabb_a.center += trans_a.translation;
        //aabb_a.half_extents = body_a.linear_velocity * pt.time * 0.5;

        let mut aabb_b = aabb_b.clone();
        aabb_b.center += trans_b.translation;
        //aabb_a.half_extents = body_b.linear_velocity * pt.time * 0.5;

        //     const BOUNDS_EPS: f32 = 0.01;
        //     bounds.expand_by_point(bounds.mins - Vec3::splat(BOUNDS_EPS));
        //     bounds.expand_by_point(bounds.maxs + Vec3::splat(BOUNDS_EPS));

        if (aabb_a.center.x - aabb_a.half_extents.x <= aabb_b.center.x + aabb_b.half_extents.x
            && aabb_a.center.x + aabb_a.half_extents.x >= aabb_b.center.x - aabb_b.half_extents.x)
            && (aabb_a.center.y - aabb_a.half_extents.y <= aabb_b.center.y + aabb_b.half_extents.y
                && aabb_a.center.y + aabb_a.half_extents.y
                    >= aabb_b.center.y - aabb_b.half_extents.y)
            && (aabb_a.center.z - aabb_a.half_extents.z <= aabb_b.center.z + aabb_b.half_extents.z
                && aabb_a.center.z + aabb_a.half_extents.z
                    >= aabb_b.center.z - aabb_b.half_extents.z)
        {
            collision_pairs.0.push(CollisionPair { a: e1, b: e2 });
        }
    }
}

// Broadphase (build potential collision pairs)
pub fn broadphase_system_old(
    query: Query<(Entity, &Body, &Transform)>,
    pt: Res<PhysicsTime>,
    mut sorted_bodies: Local<Vec<PsuedoBody>>,
    mut collision_pairs: ResMut<CollisionPairVec>,
) {
    sorted_bodies.clear();
    collision_pairs.0.clear();

    // running sweep and prune in 1 direction to test for intersection
    let axis = Vec3::ONE.normalize();
    for (entity, body, t) in query.iter() {
        let mut bounds = body.collider.bounds(t);

        // expand the bounds by the linear velocity
        bounds.expand_by_point(bounds.mins + body.linear_velocity * pt.time);
        bounds.expand_by_point(bounds.maxs + body.linear_velocity * pt.time);

        const BOUNDS_EPS: f32 = 0.01;
        bounds.expand_by_point(bounds.mins - Vec3::splat(BOUNDS_EPS));
        bounds.expand_by_point(bounds.maxs + Vec3::splat(BOUNDS_EPS));

        sorted_bodies.push(PsuedoBody {
            entity,
            value: axis.dot(bounds.mins),
            is_min: true,
        });
        sorted_bodies.push(PsuedoBody {
            entity,
            value: axis.dot(bounds.maxs),
            is_min: false,
        });
    }
    sorted_bodies.sort_unstable_by(compare_sat);

    // Build pairs
    // Now that the bodies are sorted, build the collision pairs
    for i in 0..sorted_bodies.len() {
        let a = &sorted_bodies[i];
        if !a.is_min {
            continue;
        }

        for b in sorted_bodies.iter().skip(i + 1) {
            // if we've hit the end of the a element then we're done creating pairs with a
            if b.entity == a.entity {
                break;
            }

            if !b.is_min {
                continue;
            }

            collision_pairs.0.push(CollisionPair {
                a: a.entity,
                b: b.entity,
            });
        }
    }
}

fn compare_sat(a: &PsuedoBody, b: &PsuedoBody) -> std::cmp::Ordering {
    if a.value < b.value {
        std::cmp::Ordering::Less
    } else if a.value > b.value {
        std::cmp::Ordering::Greater
    } else {
        std::cmp::Ordering::Equal
    }
}
