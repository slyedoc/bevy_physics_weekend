#![allow(dead_code)]
use crate::{
    constraints::{Constraint, ConstraintConfig, ConstraintPenetration},
    contact::Contact,
    prelude::Body,
    AddManifold,
};
use bevy::prelude::*;

const MAX_CONTACTS: usize = 4;

pub fn manifold_add_system(
    mut commands: Commands,
    bodies: Query<(Entity, &mut Body, &mut Transform)>,
    mut add_event: EventReader<AddManifold>,
    mut query: Query<&mut Manifold>,
) {
    for AddManifold(mut contact) in add_event.iter() {
        // try to find the previously existing manifold for contacts between two bodies
        let mut found = None;
        for manifold in &mut query.iter_mut() {
            #[allow(clippy::suspicious_operation_groupings)]
            let has_a =
                manifold.handle_a == contact.entity_a || manifold.handle_b == contact.entity_a;
            #[allow(clippy::suspicious_operation_groupings)]
            let has_b =
                manifold.handle_a == contact.entity_b || manifold.handle_b == contact.entity_b;
            if has_a && has_b {
                found = Some(manifold);
                break;
            }
        }

        if let Some(mut manifold) = found {
            manifold.add_contact(&bodies, &mut contact);
        } else {
            commands
                .spawn()
                .insert(Manifold::from_contact(&bodies, &mut contact))
                .insert(Name::new("Manifold"));
        }
    }
}

pub fn manifold_remove_expired(
    mut commands: Commands,
    bodies: Query<(&Body, &Transform)>,
    mut query: Query<(Entity, &mut Manifold)>,
) {
    for (e, mut manifold) in &mut query.iter_mut() {
        // remove any contacts that have drifted too far
        let mut i = 0;
        while i < manifold.num_contacts as usize {
            let contact = &manifold.contacts[i];
            unsafe {
                if let Ok((body_a, trans_a)) = bodies.get_unchecked(contact.entity_a) {
                    if let Ok((body_b, trans_b)) = bodies.get_unchecked(contact.entity_b) {
                        // get the tangential distance of the point on a and the point on b
                        let a = body_a.local_to_world(trans_a, contact.local_point_a);
                        let b = body_b.local_to_world(trans_b, contact.local_point_b);

                        let mut normal = manifold.constraints[i].normal();
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
                            i += 1;
                            continue;
                        }

                        // this contact has moved beyond its threshold and should be removed
                        for j in i..(MAX_CONTACTS - 1) {
                            manifold.constraints[j] = manifold.constraints[j + 1];
                            manifold.contacts[j] = manifold.contacts[j + 1];
                            if j >= manifold.num_contacts as usize {
                                manifold.constraints[j].clear_cached_lambda();
                            }
                        }

                        manifold.num_contacts -= 1;
                    } else {
                        warn!("Missing Entity A - Removing Manifold");
                        commands.entity(e).despawn();
                    }
                } else {
                    warn!("Missing Entity B - Removing Manifold");
                    commands.entity(e).despawn();
                }
            }
        }
        if manifold.num_contacts == 0 {
            commands.entity(e).despawn();
        }
    }
    //info!("Manifolds: {}", count);
}

#[derive(Component, Copy, Clone, Debug)]
pub struct Manifold {
    contacts: [Contact; MAX_CONTACTS],
    num_contacts: u8,
    handle_a: Entity,
    handle_b: Entity,
    constraints: [ConstraintPenetration; MAX_CONTACTS],
}

impl Manifold {
    fn new(handle_a: Entity, handle_b: Entity) -> Self {
        Self {
            contacts: [Contact::default(); MAX_CONTACTS],
            num_contacts: 0,
            handle_a,
            handle_b,
            constraints: [ConstraintPenetration::default(); MAX_CONTACTS],
        }
    }

    fn from_contact(
        bodies: &Query<(Entity, &mut Body, &mut Transform)>,
        contact: &mut Contact,
    ) -> Self {
        let mut manifold = Self::new(contact.entity_a, contact.entity_b);
        manifold.add_contact(bodies, contact);
        manifold
    }

    fn add_contact(
        &mut self,
        bodies: &Query<(Entity, &mut Body, &mut Transform)>,
        contact: &mut Contact,
    ) {
        // make sure the contact's body_a and body_b are of the correct order
        if contact.entity_a != self.handle_a || contact.entity_b != self.handle_b {
            std::mem::swap(&mut contact.local_point_a, &mut contact.local_point_b);
            std::mem::swap(&mut contact.world_point_a, &mut contact.world_point_b);
            std::mem::swap(&mut contact.entity_a, &mut contact.entity_b);
        }

        // if this contact is close to another contact then keep the old contact
        for manifold_contact in &self.contacts[0..self.num_contacts as usize] {
            unsafe {
                let (_, body_a, trans_a) = bodies.get_unchecked(manifold_contact.entity_a).unwrap();
                let (_, body_b, trans_b) = bodies.get_unchecked(manifold_contact.entity_b).unwrap();

                let old_a = body_a.local_to_world(&trans_a, manifold_contact.local_point_a);
                let old_b = body_b.local_to_world(&trans_b, manifold_contact.local_point_b);

                let (_, new_body_a, new_trans_a) = bodies.get_unchecked(contact.entity_a).unwrap();
                let (_, new_body_b, new_trans_b) = bodies.get_unchecked(contact.entity_b).unwrap();

                let new_a = new_body_a.local_to_world(&new_trans_a, contact.local_point_a);
                let new_b = new_body_b.local_to_world(&new_trans_b, contact.local_point_b);

                let aa = new_a - old_a;
                let bb = new_b - old_b;

                const DISTANCE_THRESHOLD: f32 = 0.02;
                const DISTANCE_THRESHOLD_SQ: f32 = DISTANCE_THRESHOLD * DISTANCE_THRESHOLD;
                if aa.length_squared() < DISTANCE_THRESHOLD_SQ {
                    return;
                }
                if bb.length_squared() < DISTANCE_THRESHOLD_SQ {
                    return;
                }
            }
        }

        // if we're all full on contacts then keep the contacts that are furthest away from each
        // other
        let mut new_slot = self.num_contacts as usize;
        if new_slot >= MAX_CONTACTS {
            let mut avg = Vec3::ZERO;
            avg += self.contacts[0].local_point_a;
            avg += self.contacts[1].local_point_a;
            avg += self.contacts[2].local_point_a;
            avg += self.contacts[3].local_point_a;
            avg += contact.local_point_a;
            avg *= 0.2;

            let mut min_dist = (avg - contact.local_point_a).length_squared();
            let mut new_idx = None;
            for i in 0..MAX_CONTACTS {
                let dist2 = (avg - self.contacts[i].local_point_a).length_squared();

                if dist2 < min_dist {
                    min_dist = dist2;
                    new_idx = Some(i);
                }
            }

            if let Some(new_idx) = new_idx {
                new_slot = new_idx;
            } else {
                return;
            }
        }

        self.contacts[new_slot] = *contact;

        // TODO: might be cheaper to reused existing constraint rather than memcpying a new one
        unsafe {
            let (_, _, trans_a) = bodies.get_unchecked(self.handle_a).unwrap();

            let mut normal = trans_a.rotation.inverse() * -contact.normal;
            normal = normal.normalize();
            self.constraints[new_slot] = ConstraintPenetration::new(
                ConstraintConfig {
                    handle_a: Some(contact.entity_a),
                    handle_b: Some(contact.entity_b),
                    anchor_a: contact.local_point_a,
                    anchor_b: contact.local_point_b,
                    ..ConstraintConfig::default()
                },
                normal,
            );
        }
        if new_slot == self.num_contacts as usize {
            self.num_contacts += 1;
        }
    }

    fn constraints_as_mut_slice(&mut self) -> &mut [ConstraintPenetration] {
        &mut self.constraints[0..self.num_contacts as usize]
    }

    pub fn pre_solve(
        &mut self,
        bodies: &mut Query<(Entity, &mut Body, &mut Transform)>,
        dt_sec: f32,
    ) {
        for constraint in self.constraints_as_mut_slice() {
            constraint.pre_solve(bodies, dt_sec);
        }
    }

    pub fn solve(&mut self, bodies: &mut Query<(Entity, &mut Body, &mut Transform)>) {
        for constraint in self.constraints_as_mut_slice() {
            constraint.solve(bodies);
        }
    }

    pub fn post_solve(&mut self) {
        for constraint in self.constraints_as_mut_slice() {
            constraint.post_solve();
        }
    }

    pub fn contact(&self, index: usize) -> &Contact {
        assert!(index < MAX_CONTACTS as usize);
        &self.contacts[index]
    }

    pub fn num_contacts(&self) -> usize {
        self.num_contacts as usize
    }
}

#[derive(Clone, Debug, Default)]
pub struct ManifoldCollector {
    manifolds: Vec<Manifold>,
}

impl ManifoldCollector {
    pub fn pre_solve(
        &mut self,
        bodies: &mut Query<(Entity, &mut Body, &mut Transform)>,
        dt_sec: f32,
    ) {
        for manifold in &mut self.manifolds {
            manifold.pre_solve(bodies, dt_sec);
        }
    }

    pub fn solve(&mut self, bodies: &mut Query<(Entity, &mut Body, &mut Transform)>) {
        for manifold in &mut self.manifolds {
            manifold.solve(bodies);
        }
    }

    pub fn post_solve(&mut self) {
        for manifold in &mut self.manifolds {
            manifold.post_solve();
        }
    }

    pub fn clear(&mut self) {
        self.manifolds.clear();
    }
}