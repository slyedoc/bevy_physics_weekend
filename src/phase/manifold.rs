use crate::{
    constraints::{ConstraintPenetration},
    contact::Contact,
    prelude::Body,
};
use bevy::prelude::*;

const MAX_CONTACTS: usize = 4;

pub fn manifold_remove_expired_system(
    mut commands: Commands,
    bodies: Query<(&Body, &Transform)>,
    mut query: Query<(Entity, &mut Manifold)>,
    mut remove_list: Local<Vec<usize>>,
) {
    for (e, mut manifold) in query.iter_mut() {
        remove_list.clear();

        // find contacts that have drifted too far
        for (i , cc) in manifold.contact_contraints.iter().enumerate() {
            unsafe {
                if let Ok((body_a, trans_a)) = bodies.get_unchecked(cc.contact.entity_a) {
                    if let Ok((body_b, trans_b)) = bodies.get_unchecked(cc.contact.entity_b) {
                        // get the tangential distance of the point on a and the point on b
                        let a = body_a.local_to_world(trans_a, cc.contact.local_point_a);
                        let b = body_b.local_to_world(trans_b, cc.contact.local_point_b);

                        let mut normal = cc.constraint.normal();
                        normal = trans_a.rotation * normal;

                        // calculate the tangential separation and penetration depth
                        let ab = b - a;
                        let penetration_depth = normal.dot(ab);
                        let ab_normal = normal * penetration_depth;
                        let ab_tangent = ab - ab_normal;

                        // if the tangential displacement is less than a specific threshold, it's okay to keep
                        // it.
                        const DISTANCE_THRESHOLD: f32 = 0.02;
                        if ab_tangent.length_squared() < DISTANCE_THRESHOLD * DISTANCE_THRESHOLD
                            && penetration_depth <= 0.0
                        {
                            continue;
                        }

                        // this contact has moved beyond its threshold and should be removed
                        remove_list.push(i);
                    } else {
                        //warn!("Missing Entity A - Removing Manifold");
                        commands.entity(e).despawn();
                        break;
                    }
                } else {
                    //warn!("Missing Entity B - Removing Manifold");
                    commands.entity(e).despawn();
                    break;
                }
            }
        }

        // TODO: We are only remove them later so we dont fight borrow checker on mut access
        for i in remove_list.iter().rev() {
            manifold.contact_contraints.remove(*i);
        }
        if manifold.contact_contraints.is_empty() {
            commands.entity(e).despawn();
        }
       
    }
}

#[derive(Component)]
pub struct Manifold {
    pub handle_a: Entity,
    pub handle_b: Entity,
    pub contact_contraints: Vec<ContactConstraint>,
}

pub struct ContactConstraint {
    pub contact: Contact,
    pub constraint: ConstraintPenetration
}

impl Manifold {
    pub fn new(handle_a: Entity, handle_b: Entity) -> Self {
        Self {
            contact_contraints: Vec::with_capacity(MAX_CONTACTS),
            handle_a,
            handle_b,
        }
    }
}
