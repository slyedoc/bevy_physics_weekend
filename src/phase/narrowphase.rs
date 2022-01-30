use bevy::prelude::*;

use crate::{
    colliders::{ColliderSphere, ColliderType},
    intersect,
    primitives::*,
};
// Narrowphase (perform actual collision detection)

pub fn narrowphase_system_static(
    mut broad_contacts: EventReader<BroadContact>,
    bodies: Query<(&GlobalTransform, &Body, &ColliderType)>,
    spheres: Query<&ColliderSphere>,
    //boxes: Query<&ColliderBox>,
    mut contacts: EventWriter<Contact>,
) {
    for pair in broad_contacts.iter() {
        unsafe {
            let (trans_a, body_a, shape_a) = bodies.get_unchecked(pair.a).unwrap();
            let (trans_b, body_b, shape_b) = bodies.get_unchecked(pair.b).unwrap();

            if body_a.has_infinite_mass() && body_b.has_infinite_mass() {
                continue;
            }

            match (shape_a, shape_b) {
                (ColliderType::Sphere, ColliderType::Sphere) => {
                    let sphere_a = spheres.get_unchecked(pair.a).unwrap();
                    let sphere_b = spheres.get_unchecked(pair.b).unwrap();

                    if let Some((local_point_a, local_point_b)) = intersect::sphere_sphere_static(
                        sphere_a.radius,
                        sphere_b.radius,
                        trans_a.translation,
                        trans_b.translation,
                    ) {
                        // calculate normal and separation distance
                        let normal = (trans_a.translation - trans_b.translation).normalize();
                        let ab = trans_a.translation - trans_b.translation;
                        let separation_dist = ab.length() - (sphere_a.radius + sphere_b.radius);

                        // create contact
                        contacts.send(Contact {
                            entity_a: pair.a,
                            entity_b: pair.b,
                            world_point_a: local_point_a,
                            world_point_b: local_point_b,
                            local_point_a,
                            local_point_b,
                            normal,
                            separation_dist,
                            time_of_impact: 0.0,
                        });
                    }
                }
                (_, _) => todo!(),
            }
        }
    }
}


pub fn narrowphase_system_dynamic(
    mut commands: Commands,
    mut broad_contacts: EventReader<BroadContact>,
    mut manifold_contacts: EventWriter<ManifoldContactEvent>,
    bodies: Query<(&mut GlobalTransform, &mut Body, &ColliderType)>,
    spheres: Query<&ColliderSphere>,
    //boxes: Query<&ColliderBox>,
    mut contacts: EventWriter<Contact>,
    pt: Res<crate::PhysicsTime>,
) {
    for pair in broad_contacts.iter() {
        unsafe {
            let (mut trans_a, mut body_a, shape_a) = bodies.get_unchecked(pair.a).unwrap();
            let (mut trans_b, mut body_b, shape_b) = bodies.get_unchecked(pair.b).unwrap();

            if body_a.has_infinite_mass() && body_b.has_infinite_mass() {
                continue;
            }

            
            match (shape_a, shape_b) {
                (ColliderType::Sphere, ColliderType::Sphere) => {
                    let sphere_a = spheres.get_unchecked(pair.a).unwrap();
                    let sphere_b = spheres.get_unchecked(pair.b).unwrap();

                    if let Some((world_point_a, world_point_b, time_of_impact)) =
                        intersect::sphere_sphere_dynamic(
                            sphere_a.radius,
                            sphere_b.radius,
                            trans_a.translation,
                            trans_b.translation,
                            body_a.linear_velocity,
                            body_b.linear_velocity,
                            pt.time,
                        )
                    {
                        // step bodies forward to get local space collision points
                        body_a.update(&mut trans_a, time_of_impact);
                        body_b.update(&mut trans_b, time_of_impact);

                        // convert world space contacts to local space
                        let local_point_a = body_a.world_to_local(&trans_a, world_point_a);
                        let local_point_b = body_b.world_to_local(&trans_b, world_point_b);

                        let normal = (trans_a.translation - trans_b.translation).normalize();

                        // unwind time step
                        body_a.update(&mut trans_a, -time_of_impact);
                        body_b.update(&mut trans_b, -time_of_impact);

                        // calculate the separation distance
                        let ab = trans_a.translation - trans_b.translation;
                        let separation_dist = ab.length() - (sphere_a.radius + sphere_b.radius);

                        let contact = Contact {
                            world_point_a,
                            world_point_b,
                            local_point_a,
                            local_point_b,
                            normal,
                            separation_dist,
                            time_of_impact,
                            entity_a: pair.a,
                            entity_b: pair.b,
                        };

                        if time_of_impact == 0.0 {
                            // static contact

                            manifold_contacts.send(ManifoldContactEvent(contact));
                        } else {
                            // ballistic contact
                            contacts.send(Contact {
                                entity_a: pair.a,
                                entity_b: pair.b,
                                world_point_a: local_point_a,
                                world_point_b: local_point_b,
                                local_point_a,
                                local_point_b,
                                normal,
                                separation_dist,
                                time_of_impact: 0.0,
                            });
                        }

                    }
                }
                (_, _) => todo!(),
            }

            // if let Some(contact) = contact_result {
            //     info!("Contact found between {:?} and {:?}", pair.a, pair.b);
            //     if contact.time_of_impact == 0.0 {
            //         // static contact (already touching)

            //         // try to find the previously existing manifold for contacts between two bodies
            //         let mut found = None;
            //         for manifold in &mut manifolds.iter_mut() {
            //             // we are assuming handles in both manifold and contact match
            //             if manifold.handle_a == contact.entity_a
            //                 && manifold.handle_b == contact.entity_b
            //             {
            //                 found = Some(manifold);
            //                 break;
            //             }
            //         }

            //         if let Some(mut manifold) = found {
            //             //existing manifold, add the contact to it
            //             manifold.add_contact(
            //                 &mut commands,
            //                 body_a,
            //                 trans_a,
            //                 body_b,
            //                 trans_b,
            //                 &bodies,
            //                 contact,
            //             );
            //         } else {
            //             // we don't have an existing manifold, create a new one
            //             let mut new_manifold = Manifold::new(contact.entity_a, contact.entity_b);
            //             new_manifold.add_contact(
            //                 &mut commands,
            //                 body_a,
            //                 trans_a,
            //                 body_b,
            //                 trans_b,
            //                 &bodies,
            //                 contact,
            //             );
            //             commands
            //                 .spawn()
            //                 .insert(new_manifold)
            //                 .insert(Name::new("Manifold"));
            //         }
            //     } else {
            //         // ballistic contact
            //         contacts.send(contact)
            //     }
        }
    }
}
