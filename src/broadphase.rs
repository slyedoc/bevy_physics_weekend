use bevy::prelude::*;

#[allow(unused_imports)]
use bevy::utils::Instant;
use rayon::prelude::*;

use crate::{
    body::Body,
    contact::{ContactBroad, PsuedoBody},
    PhysicsTime,
};

// Playing around with a few different solutions here


// Array based sort and sweep algorithm
// from Real-Time Collision Dection - Christer Ericon page 336
pub fn broadphase_system_array(
    mut broad: Local<usize>,
    mut contact_maybies: EventWriter<ContactBroad>,
    query: Query<(Entity, &BroadphaseAabb)>,
) {
    // TODO: Yes, we are copying the array out here, only way to sort it
    // Ideally we would keep the array around, it should already near sorted
    // but this is still far faster
    let mut list = query.iter().collect::<Vec<_>>();
    let sort_axis = broad.to_owned();

    // Sort the array on currently selected sorting axis
    // PERF: par_sort helps but this is not where the time is spent
    list.par_sort_unstable_by(|(_, a), (_, b)| {
        // Sort on minimum value along either x, y, or z axis
        let min_a = a.mins[sort_axis];
        let min_b = b.mins[sort_axis];
        if min_a < min_b {
            return std::cmp::Ordering::Less;
        }
        if min_a > min_b {
            return std::cmp::Ordering::Greater;
        }
        std::cmp::Ordering::Equal
    });

    // Sweep the array for collisions
    // This is where 90% of the time is spent
    let mut s = [0.0f32; 3];
    let mut s2 = [0.0f32; 3];
    let mut v = [0.0f32; 3];

    for (i, (a, aabb_a)) in list.iter().enumerate() {
        // Determine AABB center point
        let p = 0.5 * (aabb_a.mins + aabb_a.maxs);

        // Update sum and sum2 for computing variance of AABB centers
        for c in 0..3 {
            s[c] += p[c];
            s2[c] += p[c] * p[c];
        }
        // Test collisions against all possible overlapping AABBs following current one
        for (b, aabb_b) in list.iter().skip(i + 1) {
            // todo: + 1 may be wrong
            // Stop when tested AABBs are beyond the end of current AABB
            if aabb_b.mins[sort_axis] > aabb_a.maxs[sort_axis] {
                break;
            }
            if aabb_aabb_interect(aabb_a, aabb_b) {
                contact_maybies.send(ContactBroad { a: *a, b: *b });
            }
        }
    }

    // Compute variance (less a, for comparison unnecessary, constant factor)
    for c in 0..3 {
        v[c] = s2[c] - s[c] * s[c] / list.len() as f32;
    }

    // Update axis sorted to be the one with greatest AABB variance
    *broad = 0;
    if v[1] > v[0] {
        *broad = 1;
    }
    if v[2] > v[sort_axis] {
        *broad = 2;
    }
}

// The was the first solution from weekend series
// Broadphase (build potential collision pairs)
// sweep and prune 1d, this is from the physics weekend book
pub fn broadphase_system_weekend(
    bodies: Query<(Entity, &Body, &Transform)>,
    pt: Res<PhysicsTime>,
    mut sorted_bodies: Local<Vec<PsuedoBody>>,
    mut collison_pairs: EventWriter<ContactBroad>,
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

            collison_pairs.send(ContactBroad {
                a: a.entity,
                b: b.entity,
            });
        }
    }
}

pub fn add_broadphase_aabb(
    mut commands: Commands,
    query: Query<(Entity, &Transform, &Body), Added<Body>>,
) {
    for (e, t, b) in query.iter() {
        let bounds = b.collider.bounds(t);
        commands.entity(e).insert(BroadphaseAabb {
            mins: bounds.mins,
            maxs: bounds.maxs,
        });
    }
}

pub fn update_broadphase_array(mut query: Query<(&Transform, &Body, &mut BroadphaseAabb)>) {
    // TODO: This is a bit of a hack, to track changes
    for (trans, body, mut aabb) in query.iter_mut() {
        let bounds = body.collider.bounds(trans);
        aabb.mins = bounds.mins;
        aabb.maxs = bounds.maxs;
    }
}

#[derive(Component)]
pub struct BroadphaseAabb {
    pub mins: Vec3,
    pub maxs: Vec3,
}

// Separating Axis Test
// If there is no overlap on a particular axis,
// then the two AABBs do not intersect
#[inline]
fn aabb_aabb_interect(a: &BroadphaseAabb, b: &BroadphaseAabb) -> bool {
    if a.mins.x >= b.maxs.x {
        return false;
    }
    if a.maxs.x <= b.mins.x {
        return false;
    }

    if a.mins.y >= b.maxs.y {
        return false;
    }
    if a.maxs.y <= b.mins.y {
        return false;
    }

    if a.mins.z >= b.maxs.z {
        return false;
    }
    if a.maxs.z <= b.mins.z {
        return false;
    }

    // Overlap on all three axes, so their intersection must be non-empty
    true
}
