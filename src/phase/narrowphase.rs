use bevy::prelude::*;

use crate::{body::Body, PhysicsConfig, CollisionPairVec, intersect::intersect_dynamic, ContactVec, AddManifold, PhysicsTime};

// Narrowphase (perform actual collision detection)
pub fn narrowphase_system(
    config: Res<PhysicsConfig>,
    pt: Res<PhysicsTime>,
    query: Query<(Entity, &mut Body, &mut Transform)>,
    mut add_manifold: EventWriter<AddManifold>,
    collision_pairs: Res<CollisionPairVec>,
    mut contacts: ResMut<ContactVec>,
) {



    contacts.0.clear();
    //let t1 = Instant::now();

    // test possable contacts collisions
    for pair in collision_pairs.0.iter() {
        // SAFETY: There is no way for a and b to the same entity
        // see https://github.com/bevyengine/bevy/issues/2042
        unsafe {
            let (_, mut body_a, mut transform_a) = query.get_unchecked(pair.a).unwrap();
            let (_, mut body_b, mut transform_b) = query.get_unchecked(pair.b).unwrap();

            // skip body pairs with infinite mass
            if body_a.has_infinite_mass() && body_b.has_infinite_mass() {
                continue;
            }

            // check for intersection
            if let Some(contact) = intersect_dynamic(
                pair.a,
                &mut body_a,
                &mut transform_a,
                pair.b,
                &mut body_b,
                &mut transform_b,
                pt.time,
            ) {
                if contact.time_of_impact == 0.0 {
                    // static contact
                    add_manifold.send( AddManifold(contact));
                } else {
                    // ballistic contact
                    contacts.0.push(contact)
                }
            }
        }
    }

    // sort the times of impact from earliest to latest
    contacts.0.sort_unstable_by(|a, b| {
        if a.time_of_impact < b.time_of_impact {
            std::cmp::Ordering::Less
        } else if a.time_of_impact == b.time_of_impact {
            std::cmp::Ordering::Equal
        } else {
            std::cmp::Ordering::Greater
        }
    });
    //let t3 = Instant::now();
    //let total = (t3 - t0).as_secs_f32();
    // info!(
    //     "narrowphase_system: setup {:.1}%, loop {:.1}%, sort {:.1}%",
    //     (t1 - t0).as_secs_f32() / total * 100.0,
    //     (t2 - t1).as_secs_f32() / total * 100.0,
    //     (t3 - t2).as_secs_f32() / total * 100.0,
    // );
    //info!("Narrowphase - contacts: {}", contacts.0.len());
}