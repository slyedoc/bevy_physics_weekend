use bevy::prelude::*;

use crate::{
    colliders::{Collider, ColliderBox, ColliderSphere, ColliderType},
    intersect,
    primitives::*, PhysicsTime,
};

// Narrowphase
pub fn narrowphase_system_static(
    mut broad_contacts: EventReader<BroadContact>,
    bodies: Query<(&GlobalTransform, &Body, &ColliderType)>,
    spheres: Query<&ColliderSphere>,
    boxes: Query<&ColliderBox>,
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
                (ColliderType::Sphere, ColliderType::Box) => {
                    let sphere_a = spheres.get_unchecked(pair.a).unwrap();
                    let box_b = boxes.get_unchecked(pair.b).unwrap();
                    gjk_intersect(
                        pair,
                        sphere_a,
                        box_b,
                        trans_a,
                        trans_b,
                        body_a,
                        body_b,
                        &mut contacts,
                    );
                }
                (ColliderType::Box, ColliderType::Sphere) => {
                    let box_a = boxes.get_unchecked(pair.a).unwrap();
                    let sphere_b = spheres.get_unchecked(pair.b).unwrap();
                    gjk_intersect(
                        pair,
                        box_a,
                        sphere_b,
                        trans_a,
                        trans_b,
                        body_a,
                        body_b,
                        &mut contacts,
                    );
                }
                (ColliderType::Box, ColliderType::Box) => {
                    let box_a = boxes.get_unchecked(pair.a).unwrap();
                    let box_b = boxes.get_unchecked(pair.b).unwrap();
                    gjk_intersect(
                        pair,
                        box_a,
                        box_b,
                        trans_a,
                        trans_b,
                        body_a,
                        body_b,
                        &mut contacts,
                    );
                }
                (_, _) => todo!(),
            }
        }
    }
}

pub fn narrowphase_system_dynamic(
    mut broad_contacts: EventReader<BroadContact>,
    mut manifold_contacts: EventWriter<ManifoldContactEvent>,
    bodies: Query<(&mut GlobalTransform, &mut Body, &ColliderType)>,
    spheres: Query<&ColliderSphere>,
    boxes: Query<&ColliderBox>,
    mut contacts: EventWriter<Contact>,
    pt: Res<PhysicsTime>,
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

                        send_contact(contact, &mut manifold_contacts, &mut contacts);
                    }
                }
                (ColliderType::Sphere, ColliderType::Box) => {
                    let collider_a = spheres.get_unchecked(pair.a).unwrap();
                    let collider_b = boxes.get_unchecked(pair.b).unwrap();

                    if let Some(contact) = conservative_advancement(
                        pair,
                        &mut trans_a,
                        &mut trans_b,
                        &mut body_a,
                        &mut body_b,
                        collider_a,
                        collider_b,
                        pt.time,
                    ) {
                        send_contact(contact, &mut manifold_contacts, &mut contacts);
                    }
                }
                (ColliderType::Box, ColliderType::Sphere) => {
                    let collider_a = boxes.get_unchecked(pair.a).unwrap();
                    let collider_b = spheres.get_unchecked(pair.b).unwrap();
                    if let Some(contact) = conservative_advancement(pair,
                        &mut trans_a,
                        &mut trans_b,
                        &mut body_a,
                        &mut body_b,
                        collider_a,
                        collider_b,
                        pt.time,
                    ) {
                        send_contact(contact, &mut manifold_contacts, &mut contacts);
                    }
                },
                (ColliderType::Box, ColliderType::Box) => {
                    let collider_a = boxes.get_unchecked(pair.a).unwrap();
                    let collider_b = boxes.get_unchecked(pair.b).unwrap();

                    if let Some(contact) = conservative_advancement(
                        pair,
                        &mut trans_a,
                        &mut trans_b,
                        &mut body_a,
                        &mut body_b,
                        collider_a,
                        collider_b,
                        pt.time,
                    ) {
                        send_contact(contact, &mut manifold_contacts, &mut contacts);
                    }
                }
                (_, _) => todo!(),
            }
        }
    }
}

fn conservative_advancement(
    pair: &BroadContact,
    trans_a: &mut GlobalTransform,
    trans_b: &mut GlobalTransform,
    body_a: &mut Body,
    body_b: &mut Body,
    collider_a: &dyn Collider,
    collider_b: &dyn Collider,
    mut dt: f32,
) -> Option<Contact> {
    let mut toi = 0.0;
    let mut num_iters = 0;
    // advance the positions of the bodies until they touch or there's not time left
    while dt > 0.0 {
        // check for intersection
        const BIAS: f32 = 0.001;
        if let Some((mut world_point_a, mut world_point_b)) =
            intersect::gjk_does_intersect(collider_a, trans_a, collider_b, trans_b, BIAS)
        {
            let normal = (world_point_b - world_point_a).normalize_or_zero();
            world_point_a -= normal * BIAS;
            world_point_b += normal * BIAS;
            let contact = Contact {
                world_point_a,
                world_point_b,
                local_point_a: body_a.world_to_local(trans_a, world_point_a),
                local_point_b: body_b.world_to_local(trans_b, world_point_b),
                normal,
                separation_dist: -(world_point_a - world_point_b).length(),
                time_of_impact: toi,
                entity_a: pair.a,
                entity_b: pair.b,
            };
            body_a.update(trans_a, -toi);
            body_b.update(trans_b, -toi);

            return Some(contact);
        } else {
            let (world_point_a, world_point_b) =
                intersect::gjk_closest_points(collider_a, trans_a, collider_b, trans_b);
            let contact = Contact {
                world_point_a,
                world_point_b,
                local_point_a: body_a.world_to_local(trans_a, world_point_a),
                local_point_b: body_b.world_to_local(trans_b, world_point_b),
                normal: Vec3::ZERO,
                separation_dist: (world_point_a - world_point_b).length(),
                time_of_impact: 0.0,
                entity_a: pair.a,
                entity_b: pair.a,
            };

            // get the vector from the closest point on A to the closest point on B
            let ab = (contact.world_point_b - contact.world_point_a).normalize_or_zero();

            // project the relative velocity onto the ray of shortest distance
            let relative_velocity = body_a.linear_velocity - body_b.linear_velocity;
            let mut ortho_speed = relative_velocity.dot(ab);

            // add to the ortho_speed the maximum angular speeds of the relative shapes
            let angular_speed_a = collider_a.fastest_linear_speed(body_a.angular_velocity, ab);
            let angular_speed_b = collider_b.fastest_linear_speed(body_b.angular_velocity, -ab);
            ortho_speed += angular_speed_a + angular_speed_b;

            if ortho_speed <= 0.0 {
                break;
            }

            let time_to_go = contact.separation_dist / ortho_speed;
            if time_to_go > dt {
                break;
            }

            dt -= time_to_go;
            toi += time_to_go;
            body_a.update(trans_a, time_to_go);
            body_b.update(trans_a, time_to_go);
        };

        num_iters += 1;
        if num_iters > 10 {
            break;
        }
    }
    None
}

fn gjk_intersect(
    pair: &BroadContact,
    collider_a: &impl Collider,
    collider_b: &impl Collider,
    trans_a: &GlobalTransform,
    trans_b: &GlobalTransform,
    body_a: &Body,
    body_b: &Body,
    contacts: &mut EventWriter<Contact>,
) {
    const BIAS: f32 = 0.001;
    if let Some((mut world_point_a, mut world_point_b)) =
        intersect::gjk_does_intersect(collider_a, trans_a, collider_b, trans_b, BIAS)
    {
        let normal = (world_point_b - world_point_a).normalize_or_zero();
        world_point_a -= normal * BIAS;
        world_point_b += normal * BIAS;

        contacts.send(Contact {
            entity_a: pair.a,
            entity_b: pair.b,
            world_point_a,
            world_point_b,
            local_point_a: body_a.world_to_local(trans_a, world_point_a),
            local_point_b: body_b.world_to_local(trans_b, world_point_b),
            normal,
            separation_dist: -(world_point_a - world_point_b).length(),
            time_of_impact: 0.0,
        });
    }
}

fn send_contact(
    contact: Contact,
    manifold_contacts: &mut EventWriter<ManifoldContactEvent>,
    contacts: &mut EventWriter<Contact>,
) {
    if contact.time_of_impact == 0.0 {
        // static contact
        manifold_contacts.send(ManifoldContactEvent(contact));
    } else {
        // ballistic contact
        contacts.send(contact);
    }
}
