use crate::shapes::*;
use bevy::prelude::*;

use bevy_inspector_egui::Inspectable;

#[derive(Component, Copy, Clone, Debug, Inspectable)]
pub struct Body {
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
    pub inv_mass: f32,
    pub shape: PyhsicsShape,
    #[inspectable(min = 0.0, max = 1.0)]
    pub elasticity: f32,

    #[inspectable(min = 0.0, max = 1.0)]
    pub friction: f32,
}

impl Default for Body {
    fn default() -> Self {
        Self {
            linear_velocity: Vec3::default(),
            angular_velocity: Vec3::default(),
            inv_mass: 1.0,
            shape: PyhsicsShape::default(),
            elasticity: 0.5,
            friction: 0.5,
        }
    }
}

impl Body {
    pub fn get_centre_of_mass_local(&self) -> Vec3 {
        self.shape.centre_of_mass()
    }
    pub fn get_centre_of_mass_world(&self, t: Transform) -> Vec3 {
        t.translation + t.rotation * self.get_centre_of_mass_local()
    }

    pub fn get_inv_inertia_tensor_local(&self) -> Mat3 {
        self.shape.inertia_tensor().inverse() * self.inv_mass
    }

    pub fn get_inv_inertia_tensor_world(&self, t: Transform) -> Mat3 {
        let inv_inertia_tensor = self.get_inv_inertia_tensor_local();
        let orientation = Mat3::from_quat(t.rotation);
        orientation * inv_inertia_tensor * orientation.transpose()
    }

    pub fn apply_impulse(&mut self, impulse_point: Vec3, impulse: Vec3, t: Transform) {
        if self.inv_mass == 0.0 {
            return;
        }
        // impulse_point is in world space location of the applied impulse
        // impulse is in world space direction and magnitude of the impulse
        self.apply_impulse_linear(impulse);

        let position = self.get_centre_of_mass_world(t); // aplying impluses must produce torgues though the center of mass
        let r = impulse_point - position;
        let dl = r.cross(impulse); // this is in world space
        self.apply_impulse_angular(dl, t);
    }

    pub fn apply_impulse_linear(&mut self, impulse: Vec3) {
        if self.inv_mass == 0.0 {
            return;
        }
        // p = mv
        // dp = m dv = J
        // => dv = J / m
        self.linear_velocity += impulse * self.inv_mass;
    }
    pub fn apply_impulse_angular(&mut self, impulse: Vec3, t: Transform) {
        if self.inv_mass == 0.0 {
            return;
        }

        // L = I w = r x p
        // dL = I dw = r x J
        // => dw = I^-1 * (r x J)
        self.angular_velocity += self.get_inv_inertia_tensor_world(t) * impulse;

        // clamp angular_velocity - 30 rad/s is fast enough for us
        const MAX_ANGULAR_SPEED: f32 = 30.0;
        const MAX_ANGULAR_SPEED_SQ: f32 = MAX_ANGULAR_SPEED * MAX_ANGULAR_SPEED;
        if self.angular_velocity.length_squared() > MAX_ANGULAR_SPEED_SQ {
            self.angular_velocity = self.angular_velocity.normalize() * MAX_ANGULAR_SPEED;
        }
    }
}
