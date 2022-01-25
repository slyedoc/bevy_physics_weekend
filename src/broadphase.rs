use bevy::{prelude::*, render::primitives::Aabb};

use crate::{
    body::Body,
    contact::{ContactMaybe, PsuedoBody},
    PhysicsTime, bounds::Bounds,
};

// Broadphase (build potential collision pairs)
// Playing around with a few different solutions here

// sweep and prune 1d, this is from the physics weekend book
pub fn broadphase_system(
    bodies: Query<(Entity, &Body, &Transform)>,
    pt: Res<PhysicsTime>,
    mut sorted_bodies: Local<Vec<PsuedoBody>>,
    mut collison_pairs: EventWriter<ContactMaybe>,
) {
    sorted_bodies.clear();

    // running sweep and prune in 1 direction to test for intersection
    let axis = Vec3::ONE.normalize();
    for (entity, body, t) in bodies.iter() {

        // copy the bounds so we can expand it
        let mut bounds = body.collider.bounds(t);

        // expand the bounds by the linear velocity
        bounds.expand_by_point(bounds.mins + body.linear_velocity * pt.time);
        bounds.expand_by_point(bounds.maxs + body.linear_velocity * pt.time);

        const BOUNDS_EPS: f32 = 0.01;
        bounds.expand_by_point(bounds.mins - Vec3::splat(BOUNDS_EPS));
        bounds.expand_by_point(bounds.maxs + Vec3::splat(BOUNDS_EPS));

        // find the min and max of the bounds in the direction of the axis
        // TODO: try out vec3a here
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

    // Now that the bodies are sorted, we can iterate through them and send them to narrow phase
    // if they overlap
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

fn broadphase_system_3d(
    //mut sort_axis: Local<usize> // Specifies axis (0/1/2) to sort on (here arbitrarily initialized)
) {
    // sort_axis = 0;
    // if v[1] > v[0] {
    //     sort_axis = 1;
    // }
    // if v[2] > v[sort_axis] {
    //     sort_axis = 2;
    // } 
}

// // Comparison function for qsort. Given two arguments A and B must return a
// // value of less than zero if A < B, zero if A = B, and greater than zero if A>B
// fn cmpAABBs(a: &Bounds, b: &Bounds) -> i32 {
//     // Sort on minimum value along either x, y, or z (specified in gSortAxis)
//     let minA = a.mins.x; //[sort_axis]
//     let minB = a.mins.x; //[sort_axis]
//     if minA < minB {
//         return -1;
//     }
//     if minA > minB {
//         return 1;
//     }
//     0
//}