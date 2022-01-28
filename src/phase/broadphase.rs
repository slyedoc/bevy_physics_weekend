use bevy::prelude::*;
use bevy::tasks::*;

use bevy::tasks::ParallelSlice;
#[allow(unused_imports)]
use bevy::utils::Instant;

use crate::primitives::*;

// Playing around with a few different solutions here

// Array based sort and sweep algorithm
// from Real-Time Collision Dection - Christer Ericon page 336
pub fn broadphase_system(
    mut broad_contacts: EventWriter<BroadContact>,
    query: Query<(Entity, &Aabb)>,
    mut sort_axis_local: Local<usize>,
    mut possable_contacts_local: Local<Vec<BroadContact>>,
    pool: Res<ComputeTaskPool>,
) {
    possable_contacts_local.clear();

    // TODO: Yes, we are copying the array out here, only way to sort it
    // Ideally we would keep the array around, it should already near sorted
    // but this is still far faster
    let mut list = query.iter().collect::<Vec<_>>();
    let sort_axis = sort_axis_local.to_owned();

    // Sort the array on currently selected sorting axis
    // TODO: par_sort helps a little, but mostly so harmful on computer with less cores that its not worth it
    // this is not where the time is spent
    list.sort_unstable_by(|(_, a), (_, b)| {
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

            // build up a list we need to do aabb collision detection on
            possable_contacts_local.push(BroadContact { a: *a, b: *b });
            //broad_contacts.send(BroadContact { a: *a, b: *b });
        }
    }

    // Compute variance (less a, for comparison unnecessary, constant factor)
    for c in 0..3 {
        v[c] = s2[c] - s[c] * s[c] / list.len() as f32;
    }

    // Update axis sorted to be the one with greatest AABB variance
    *sort_axis_local = 0;
    if v[1] > v[0] {
        *sort_axis_local = 1;
    }
    if v[2] > v[sort_axis] {
        *sort_axis_local = 2;
    }

    // filter possable contacts
    // We still have far to many contacts, filter them by doing aabb collision detection

    let t0 = Instant::now();
    //#[cfg(feature = "simd")]
    {
        let simd_contacts = possable_contacts_local
            .par_chunk_map(&pool, 16384, |main_chuck| {
                let mut results = Vec::new();

                let iter = main_chuck.chunks_exact(4);

                for c in iter.remainder() {
                    unsafe {
                        let (_, aabb_a) = query.get_unchecked(c.a).unwrap();
                        let (_, aabb_b) = query.get_unchecked(c.b).unwrap();
                        if aabb_aabb_intersect(aabb_a, aabb_b) {
                            results.push(*c);
                        }
                    }
                }
                for (i, chunk) in iter.enumerate() {
                    unsafe {
                        // 4 by 4 aabb test
                        // setup 4 aabbs pairs to be tested
                        let (_, aabb_1a) = query.get_unchecked(chunk[0].a).unwrap();
                        let (_, aabb_1b) = query.get_unchecked(chunk[0].b).unwrap();
                        let (_, aabb_2a) = query.get_unchecked(chunk[1].a).unwrap();
                        let (_, aabb_2b) = query.get_unchecked(chunk[1].b).unwrap();
                        let (_, aabb_3a) = query.get_unchecked(chunk[2].a).unwrap();
                        let (_, aabb_3b) = query.get_unchecked(chunk[2].b).unwrap();
                        let (_, aabb_4a) = query.get_unchecked(chunk[3].a).unwrap();
                        let (_, aabb_4b) = query.get_unchecked(chunk[3].b).unwrap();

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

                        if i == 0 {
                            //     info!("1a min x: {}, 1b min x: {}", aabb_1a.mins.x, aabb_1b.mins.x);
                            //     info!("1a max x: {}, 1b max x: {}", aabb_1a.maxs.x, aabb_1b.maxs.x);
                            //     info!("min1x {}, min2x {}", min1x, min2x);
                            //     info!("max1x {}, max2x {}", max1x, max2x);
                            //     info!("ax {}, bx {}", ax, bx);
                            //     info!("t1 - {:?}, t2 - {:?}, t3 - {:?}",
                            //         t1, t2, t3
                            //     );
                            //  info!("result - {:?}",
                            //      result
                            //  );
                        }

                        let intersect: [bool; 4] = result.into();
                        for i in 0..4 {
                            if intersect[i] {
                                results.push(chunk[i]);
                            }
                        }
                    }
                }

                results
            })
            .into_iter()
            .flatten();
        broad_contacts.send_batch(simd_contacts);
    }

    let t1 = Instant::now();
    //#[cfg(not(feature = "simd"))]
    {
        let final_contacts = possable_contacts_local
            .par_chunk_map(&pool, 16384, |chunk| {
                let mut results = Vec::new();
                for c in chunk {
                    unsafe {
                        let (_, aabb_a) = query.get_unchecked(c.a).unwrap();
                        let (_, aabb_b) = query.get_unchecked(c.b).unwrap();
                        if aabb_aabb_intersect(aabb_a, aabb_b) {
                            results.push(*c);
                        }
                    }
                }
                results
            })
            .into_iter()
            .flatten();
        //broad_contacts.send_batch(final_contacts);
    }
    let t2 = Instant::now();
    
    let simd = t1.duration_since(t0);
    let old = t2.duration_since(t1);
    let diff = simd.div_duration_f32(old);
    match diff > 1.0 {
        true => {
            warn!("simb: {:?}, old: {:?}, diff {}", simd, old, 1.0 - diff);
        }
        false => {
            info!("simb: {:?}, old: {:?}, diff {}", simd, old, 1.0 - diff);
        }
    }
    
    
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

// Separating Axis Test
// If there is no overlap on a particular axis,
// then the two AABBs do not intersect
#[inline]
fn aabb_aabb_intersect(a: &Aabb, b: &Aabb) -> bool {
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
