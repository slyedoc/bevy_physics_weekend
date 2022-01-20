use bevy::prelude::*;

use super::CollisionPair;
use crate::{body::Body, CollisionPairVec, PhysicsTime};
#[derive(Copy, Clone, Debug)]
pub struct PsuedoBody {
    entity: Entity,
    value: f32,
    is_min: bool,
}

// Broadphase (build potential collision pairs)
pub fn broadphase_system(
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