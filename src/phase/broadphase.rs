use bevy::prelude::*;

use crate::{primitives::*, intersect, bounds::aabb::Aabb};

// The board phase is responsible for pruning the search space of possable collisions
// I have tried different approaches, and I am sure I will try a few more
// So far this simple approach has been the fastest
// TODO: Figure out way to search two axis thats actually faster or bite the bullet and try some space partitioning
pub fn broadphase_system(
    mut broad_contacts: EventWriter<BroadContact>,
    query: Query<(Entity, &Aabb, &GlobalTransform)>,
) {
    // TODO: Yes, we are copying the array out here, only way to sort it
    // Ideally we would keep the array around, it should already near sorted
    let mut list = query.iter()
        .map(|(e, aabb, t)| {
             (e, Aabb::from_extents(t.translation + aabb.minimums(), t.translation + aabb.maximums()) )
        })
        .collect::<Vec<_>>();
    
    // Sort the array on currently selected sorting axis
    list.sort_unstable_by(cmp_x_axis);

    // Sweep the array for collisions
    for (i, (a, aabb_a)) in list.iter().enumerate() {
        // Test collisions against all possible overlapping AABBs following current one
        for (b, aabb_b) in list.iter().skip(i + 1) {
            // Stop when tested AABBs are beyond the end of current AABB
            if aabb_b.minimums().x > aabb_a.maximums().x {
                break;
            }
            if intersect::aabb_aabb_intersect(aabb_a, aabb_b) {
                broad_contacts.send(BroadContact { a: *a, b: *b });
            }
        }

    }
}

fn cmp_x_axis( a: &(Entity, Aabb), b: &(Entity, Aabb)) -> std::cmp::Ordering {
    // Sort on minimum value along either x, y, or z axis
    let min_a = a.1.minimums().x;
    let min_b = b.1.minimums().x;
    if min_a < min_b {
        return std::cmp::Ordering::Less;
    }
    if min_a > min_b {
        return std::cmp::Ordering::Greater;
    }
    std::cmp::Ordering::Equal
}

fn cmp_y_axis( a: &(Entity, Aabb), b: &(Entity, Aabb)) -> std::cmp::Ordering {
    // Sort on minimum value along either x, y, or z axis
    let min_a = a.1.minimums().y;
    let min_b = b.1.minimums().y;
    if min_a < min_b {
        return std::cmp::Ordering::Less;
    }
    if min_a > min_b {
        return std::cmp::Ordering::Greater;
    }
    std::cmp::Ordering::Equal
}

fn cmp_z_axis( a: &(Entity, Aabb), b: &(Entity, Aabb)) -> std::cmp::Ordering {
    // Sort on minimum value along either x, y, or z axis
    let min_a = a.1.minimums().z;
    let min_b = b.1.minimums().z;
    if min_a < min_b {
        return std::cmp::Ordering::Less;
    }
    if min_a > min_b {
        return std::cmp::Ordering::Greater;
    }
    std::cmp::Ordering::Equal
}


// TODO: Was trying to parrallize part of the sweep, its faster most frames but not always, and not that much faster,
// and pikey
#[cfg(feature = "disabled")]
fn find_aabb_intersects(possible_contacts: &[((Entity, &Aabb), (Entity, &Aabb))], pool: Res<ComputeTaskPool>, chunk_size: usize) -> impl Iterator<Item = BroadContact> {
    possible_contacts
        .par_chunk_map(&pool, chunk_size, |chunk| {
            let mut results = vec![];
            for ((a, aabb_a), (b, aabb_b)) in chunk.iter() {
                if intersect::aabb_aabb_intersect(aabb_a, aabb_b) {
                    results.push(BroadContact { a: *a, b: *b });
                }
            }
            results
        })
        .into_iter()
        .flatten()
}


// 4 by 4 Aabb simd test
// This works, but it was about 10-30% slower in my testing, will come back to this later
#[cfg(feature = "disabled")]
fn find_aabb_simd_intersects(possible_contacts: &[((Entity, &Aabb), (Entity, &Aabb))], pool: &Res<ComputeTaskPool>, chunk_size: usize) -> impl Iterator<Item = BroadContact> {
    possible_contacts
         .par_chunk_map(pool, chunk_size, |main_chuck| {
             let iter = main_chuck.chunks_exact(4);
             let mut results = vec![];

             // go ahead and process any tests that wont fit our simd test
            for ((a, aabb_a), (b, aabb_b)) in iter.remainder() {
                if intersect::aabb_aabb_intersect(aabb_a, aabb_b) {
                    results.push(BroadContact { a: *a, b: *b });
                }
            }
            for chunk in iter {
                // 4 by 4 aabb test
                // setup 4 aabbs pairs to be tested
                let ((_, aabb_1a), (_, aabb_1b)) = chunk[0];
                let ((_, aabb_2a), (_, aabb_2b)) = chunk[1];
                let ((_, aabb_3a), (_, aabb_3b)) = chunk[2];
                let ((_, aabb_4a), (_, aabb_4b)) = chunk[3];

                // breaking aabb into Vec4s
                // Group 1
                let min1x = Vec4::new(
                    aabb_1a.mins.x,
                    aabb_2a.mins.x,
                    aabb_3a.mins.x,
                    aabb_4a.mins.x,
                );
                let min1y = Vec4::new(
                    aabb_1a.mins.y,
                    aabb_2a.mins.y,
                    aabb_3a.mins.y,
                    aabb_4a.mins.y,
                );
                let min1z = Vec4::new(
                    aabb_1a.mins.z,
                    aabb_2a.mins.z,
                    aabb_3a.mins.z,
                    aabb_4a.mins.z,
                );

                let max1x = Vec4::new(
                    aabb_1a.maxs.x,
                    aabb_2a.maxs.x,
                    aabb_3a.maxs.x,
                    aabb_4a.maxs.x,
                );
                let max1y = Vec4::new(
                    aabb_1a.maxs.y,
                    aabb_2a.maxs.y,
                    aabb_3a.maxs.y,
                    aabb_4a.maxs.y,
                );
                let max1z = Vec4::new(
                    aabb_1a.maxs.z,
                    aabb_2a.maxs.z,
                    aabb_3a.maxs.z,
                    aabb_4a.maxs.z,
                );

                // Group 2
                let min2x = Vec4::new(
                    aabb_1b.mins.x,
                    aabb_2b.mins.x,
                    aabb_3b.mins.x,
                    aabb_4b.mins.x,
                );
                let min2y = Vec4::new(
                    aabb_1b.mins.y,
                    aabb_2b.mins.y,
                    aabb_3b.mins.y,
                    aabb_4b.mins.y,
                );
                let min2z = Vec4::new(
                    aabb_1b.mins.z,
                    aabb_2b.mins.z,
                    aabb_3b.mins.z,
                    aabb_4b.mins.z,
                );

                let max2x = Vec4::new(
                    aabb_1b.maxs.x,
                    aabb_2b.maxs.x,
                    aabb_3b.maxs.x,
                    aabb_4b.maxs.x,
                );
                let max2y = Vec4::new(
                    aabb_1b.maxs.y,
                    aabb_2b.maxs.y,
                    aabb_3b.maxs.y,
                    aabb_4b.maxs.y,
                );
                let max2z = Vec4::new(
                    aabb_1b.maxs.z,
                    aabb_2b.maxs.z,
                    aabb_3b.maxs.z,
                    aabb_4b.maxs.z,
                );

                let ax = min1x.max(min2x);
                let bx = max1x.min(max2x);
                let ay = min1y.max(min2y);
                let by = max1y.min(max2y);
                let az = min1z.max(min2z);
                let bz = max1z.min(max2z);

                let t1 = ax.cmple(bx);
                let t2 = ay.cmple(by);
                let t3 = az.cmple(bz);
                let t4 = t1 & t2;
                let result = t3 & t4;

                let intersect: [bool; 4] = result.into();
                for i in 0..4 {
                    if intersect[i] {
                        results.push(BroadContact {
                            a: chunk[i].0 .0,
                            b: chunk[i].1 .0,
                        });
                    }
                }
            }
            results
        })
        .into_iter()
        .flatten()
}




// The was the first solution from weekend series
// Broadphase (build potential collision pairs)
// sweep and prune 1d, this is from the physics weekend book
#[cfg(feature = "disabled")]
pub fn broadphase_system_weekend(
    bodies: Query<(Entity, &Body, &Transform)>,
    pt: Res<PhysicsTime>,
    mut sorted_bodies: Local<Vec<PsuedoBody>>,
    mut collison_pairs: EventWriter<BroadContact>,
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
