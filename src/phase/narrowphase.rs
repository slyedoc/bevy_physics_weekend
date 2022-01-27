use bevy::prelude::*;

use crate::{intersect, primitives::*};

pub fn narrowphase_breakout_system(
    mut commands: Commands,
    mut broad_contacts: EventReader<BroadContact>,
    mut sphere_sphere: EventWriter<BroadSphereSphere>,
    mut sphere_box: EventWriter<BroadSphereBox>,
    mut box_box: EventWriter<BroadBoxBox>,
    mut contacts: EventWriter<Contact>,
    bodies: Query<(&Body, &ColliderType)>,
) {
    for pair in broad_contacts.iter() {
        //SAFETY: There is no way for a and b to the same entity
        //see https://github.com/bevyengine/bevy/issues/2042
        unsafe {
            let (body_a, shape_a) = bodies.get_unchecked(pair.a).unwrap();
            let (body_b, shape_b) = bodies.get_unchecked(pair.b).unwrap();

            if body_a.has_infinite_mass() && body_b.has_infinite_mass() {
                continue;
            }

            match (shape_a, shape_b) {
                (ColliderType::Sphere, ColliderType::Sphere) => sphere_sphere.send(BroadSphereSphere(*pair)),
                (ColliderType::Sphere, ColliderType::Box) => sphere_box.send(BroadSphereBox(*pair)),
                
                // swap order so we dont need more services
                (ColliderType::Box, ColliderType::Sphere) => sphere_sphere.send(BroadSphereSphere(BroadContact {
                        a: pair.b,
                        b: pair.a,
                    })),
                
                (ColliderType::Box, ColliderType::Box) => box_box.send(BroadBoxBox(*pair)),
                // (ShapeType::Box, ShapeType::Convex) => todo!(),
                // (ShapeType::Convex, ShapeType::Sphere { radius }) => todo!(),
                // (ShapeType::Convex, ShapeType::Box) => todo!(),
                (_, _) => todo!(),
            }
        }
    }
}

pub fn narrow_phase_sphere_sphere(
    mut pairs: EventReader<BroadSphereSphere>,
    spheres: Query<(&GlobalTransform, &ColliderSphere)>,
    mut contacts: EventWriter<Contact>,
) {
    for pair in pairs.iter() {
        unsafe {
            let (trans_a, sphere_a) = spheres.get_unchecked(pair.0.a).unwrap();
            let (trans_b, sphere_b) = spheres.get_unchecked(pair.0.b).unwrap();

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
                    entity_a: pair.0.a,
                    entity_b: pair.0.b,
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
}


pub fn narrow_phase_sphere_box(
    mut pairs: EventReader<BroadSphereBox>,
    spheres: Query<(&GlobalTransform, &Body, &ColliderSphere)>,
    boxes: Query<(&GlobalTransform, &Body, &ColliderBox)>,
    mut contacts: EventWriter<Contact>,
) {
    for pair in pairs.iter() {
        unsafe {
            let (trans_a, body_a, sphere_a) = spheres.get_unchecked(pair.0.a).unwrap();
            let (trans_b, body_b, box_b) = boxes.get_unchecked(pair.0.b).unwrap();

            const BIAS: f32 = 0.001;
            if let Some((mut world_point_a, mut world_point_b)) =
                intersect::gjk_does_intersect(sphere_a, trans_a, box_b, trans_b, BIAS)
            {
                let normal = (world_point_b - world_point_a).normalize_or_zero();
                world_point_a -= normal * BIAS;
                world_point_b += normal * BIAS;
                
                contacts.send(
                    Contact {
                        world_point_a,
                        world_point_b,
                        local_point_a: body_a.world_to_local(trans_a, world_point_a),
                        local_point_b: body_b.world_to_local(trans_b, world_point_b),
                        normal,
                        separation_dist: -(world_point_a - world_point_b).length(),
                        time_of_impact: 0.0,
                        entity_a: pair.0.a,
                        entity_b: pair.0.b,
                    })
            }
        }
    }
}


pub fn narrow_phase_box_box(
    mut pairs: EventReader<BroadSphereBox>,    
    boxes: Query<(&GlobalTransform, &Body, &ColliderBox)>,
    mut contacts: EventWriter<Contact>,
) {
    for pair in pairs.iter() {
        unsafe {
            let (trans_a, body_a, box_a) = boxes.get_unchecked(pair.0.a).unwrap();
            let (trans_b, body_b, box_b) = boxes.get_unchecked(pair.0.b).unwrap();

            const BIAS: f32 = 0.001;
            if let Some((mut world_point_a, mut world_point_b)) =
                intersect::gjk_does_intersect(box_a, trans_a, box_b, trans_b, BIAS)
            {
                let normal = (world_point_b - world_point_a).normalize_or_zero();
                world_point_a -= normal * BIAS;
                world_point_b += normal * BIAS;
                
                contacts.send(
                    Contact {
                        world_point_a,
                        world_point_b,
                        local_point_a: body_a.world_to_local(trans_a, world_point_a),
                        local_point_b: body_b.world_to_local(trans_b, world_point_b),
                        normal,
                        separation_dist: -(world_point_a - world_point_b).length(),
                        time_of_impact: 0.0,
                        entity_a: pair.0.a,
                        entity_b: pair.0.b,
                    })
            }
        }
    }
}


#[cfg(feature = "dynamic")]
use crate::PhysicsTime;

#[cfg(feature = "dynamic")]
// Narrowphase (perform actual collision detection)
pub fn narrowphase_system_dynamic(
    mut commands: Commands,
    pt: Res<PhysicsTime>,
    bodies: Query<(&Body, &Transform)>,
    mut contacts: EventWriter<Contact>,
    mut manifolds: Query<&mut Manifold>,
    mut contact_broad: EventReader<BroadContact>,
) {
    // test possable contacts collisions
    for pair in contact_broad.iter() {
        // SAFETY: There is no way for a and b to the same entity
        // see https://github.com/bevyengine/bevy/issues/2042
        unsafe {
            let (body_a, transform_a) = bodies.get_unchecked(pair.a).unwrap();
            let (body_b, transform_b) = bodies.get_unchecked(pair.b).unwrap();

            // skip body pairs with infinite mass
            if body_a.has_infinite_mass() && body_b.has_infinite_mass() {
                continue;
            }

            // check for intersection, and make sure entity a is less b, will save checks later
            let contact_result = if pair.a <= pair.b {
                intersect_dynamic(
                    pair.a,
                    body_a,
                    transform_a,
                    pair.b,
                    body_b,
                    transform_b,
                    pt.time,
                )
            } else {
                intersect_dynamic(
                    pair.b,
                    body_b,
                    transform_b,
                    pair.a,
                    body_a,
                    transform_a,
                    pt.time,
                )
            };

            if let Some(contact) = contact_result {
                info!("Contact found between {:?} and {:?}", pair.a, pair.b);
                if contact.time_of_impact == 0.0 {
                    // static contact (already touching)

                    // try to find the previously existing manifold for contacts between two bodies
                    let mut found = None;
                    for manifold in &mut manifolds.iter_mut() {
                        // we are assuming handles in both manifold and contact match
                        if manifold.handle_a == contact.entity_a
                            && manifold.handle_b == contact.entity_b
                        {
                            found = Some(manifold);
                            break;
                        }
                    }

                    if let Some(mut manifold) = found {
                        //existing manifold, add the contact to it
                        manifold.add_contact(
                            &mut commands,
                            body_a,
                            transform_a,
                            body_b,
                            transform_b,
                            &bodies,
                            contact,
                        );
                    } else {
                        // we don't have an existing manifold, create a new one
                        let mut new_manifold = Manifold::new(contact.entity_a, contact.entity_b);
                        new_manifold.add_contact(
                            &mut commands,
                            body_a,
                            transform_a,
                            body_b,
                            transform_b,
                            &bodies,
                            contact,
                        );
                        commands
                            .spawn()
                            .insert(new_manifold)
                            .insert(Name::new("Manifold"));
                    }
                } else {
                    // ballistic contact
                    contacts.send(contact)
                }
            }
        }
    }
}
