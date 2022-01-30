use crate::
    constraints::ConstraintPenetration
;
use bevy::prelude::*;

use super::{Contact, Body};

const MAX_CONTACTS: usize = 4;

#[derive(Component)]
pub struct Manifold {
    pub handle_a: Entity,
    pub handle_b: Entity,
    pub contact_contraints: Vec<ManifoldConstraint>,
}

pub struct ManifoldConstraint {
    pub contact: Contact,
    pub constraint_entity: Entity,
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

pub fn manifold_remove_expired_system(
    mut commands: Commands,
    bodies: Query<(&Body, &GlobalTransform)>,
    mut manifolds: Query<(Entity, &mut Manifold)>,
    constraints: Query<&ConstraintPenetration>,
) {
    for (e, mut manifold) in manifolds.iter_mut() {
        unsafe {
            let a = bodies.get_unchecked(manifold.handle_a);
            let b = bodies.get_unchecked(manifold.handle_b);

            if a.is_err() || b.is_err() {
                warn!(
                    "Manifold invalid entity handles, handle_a: {:?}, handle_b: {:?}",
                    manifold.handle_a, manifold.handle_b
                );
                for cc in &manifold.contact_contraints {
                    commands.entity(cc.constraint_entity).despawn();
                }
                commands.entity(e).despawn();
                continue;
            }

            let (body_a, trans_a) = a.unwrap();
            let (body_b, trans_b) = b.unwrap();

            if manifold.contact_contraints.len() >= MAX_CONTACTS {
                // find contacts that have drifted too far
                manifold.contact_contraints.retain(|cc| {
                    // get the tangential distance of the point on a and the point on b
                    let pos_a = body_a.local_to_world(trans_a, cc.contact.local_point_a);
                    let pos_b = body_b.local_to_world(trans_b, cc.contact.local_point_b);

                    let contraint = constraints.get(cc.constraint_entity).unwrap();
                    let normal = trans_a.rotation * contraint.normal();

                    // calculate the tangential separation and penetration depth
                    let ab = pos_b - pos_a;
                    let penetration_depth = normal.dot(ab);
                    let ab_normal = normal * penetration_depth;
                    let ab_tangent = ab - ab_normal;

                    // if the tangential displacement is less than a specific threshold, it's okay to keep
                    // it.
                    const DISTANCE_THRESHOLD: f32 = 0.02;
                    if ab_tangent.length_squared() < DISTANCE_THRESHOLD * DISTANCE_THRESHOLD
                        && penetration_depth <= 0.0
                    {
                        return true;
                    }

                    // this contact has moved beyond its threshold and should be removed
                    warn!("Remove Constraint: {:?}", cc.constraint_entity);
                    commands.entity(cc.constraint_entity).despawn();
                    false
                });
            }
        }

        // Clean up self if empty
        // if manifold.contact_contraints.is_empty() {
        //     warn!("Manifold Empty: {:?}", e);
        //     commands.entity(e).despawn();
        // }
    }
}
