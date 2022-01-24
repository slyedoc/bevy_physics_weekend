use bevy::{prelude::*, render::primitives::Aabb};

use crate::{
    body::Body,
    contact::{ContactMaybe, PsuedoBody},
    PhysicsTime,
};

// Broadphase (build potential collision pairs)
// sweep and prune 1d
pub fn broadphase_system(
    query: Query<(Entity, &Body, &Transform)>,
    pt: Res<PhysicsTime>,
    mut sorted_bodies: Local<Vec<PsuedoBody>>,
    mut collison_pairs: EventWriter<ContactMaybe>,
) {
    sorted_bodies.clear();

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
    sorted_bodies.sort_unstable_by(PsuedoBody::compare_sat);

    // Build pairs
    // Now that the bodies are sorted, build the collision pairs
    for (i, a) in sorted_bodies.iter().enumerate() {
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

            collison_pairs.send(ContactMaybe {
                a: a.entity,
                b: b.entity,
            });
        }
    }
}



// Broadphase (build potential collision pairs)
// wanted to test using the render Aabb
// TODO: Only a test for now
pub fn broadphase_system_aabb(
    query: Query<(Entity, &Transform, &Aabb)>,
    //pt: Res<PhysicsTime>,
    mut contact_maybies: EventWriter<ContactMaybe>,
) {
    let mut iter = query.iter_combinations();
    while let Some([(e1, trans_a, aabb_a), (e2, trans_b, aabb_b)]) = iter.fetch_next() {
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
            contact_maybies.send(ContactMaybe { a: e1, b: e2 });
        }
    }
}

