use std::sync::Arc;

use crate::shapes::*;
use bevy::prelude::*;

#[derive(Component)]
pub struct Body {
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
    pub inv_mass: f32,
    pub shape: Shape,
    pub elasticity: f32, // min = 0.0, max = 1.0
    pub friction: f32, // min = 0.0, max = 1.0
}

impl Default for Body {
    fn default() -> Self {
        Self {
            linear_velocity: Vec3::default(),
            angular_velocity: Vec3::default(),
            inv_mass: 1.0,
            shape: Shape::default() ,
            elasticity: 0.5,
            friction: 0.5,
        }
    }
}

impl Body {


    pub fn get_centre_of_mass_world(&self, t: &Transform) -> Vec3 {
        t.translation + t.rotation * self.shape.centre_of_mass()
    }

    pub fn world_to_local(&self, t: &Transform, world_point: Vec3) -> Vec3 {
        let tmp = world_point - self.get_centre_of_mass_world(t);
        let inv_orientation = t.rotation.conjugate();
        let body_space = inv_orientation * tmp;
        body_space
    }

    pub fn local_to_world(&self, t: &Transform, body_point: Vec3) -> Vec3 {
        let world_point = self.get_centre_of_mass_world(t) + t.rotation * body_point;
        world_point
    }

    pub fn get_inv_inertia_tensor_local(&self) -> Mat3 {
        self.shape.inertia_tensor().inverse() * self.inv_mass
    }

    pub fn get_inv_inertia_tensor_world(&self, t: &Transform) -> Mat3 {
        let inv_inertia_tensor = self.get_inv_inertia_tensor_local();
        let orientation = Mat3::from_quat(t.rotation);
        orientation * inv_inertia_tensor * orientation.transpose()
    }

    pub fn apply_impulse(&mut self, impulse_point: Vec3, impulse: Vec3, t: &mut Transform) {
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
    pub fn apply_impulse_angular(&mut self, impulse: Vec3, t: &mut Transform) {
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

    pub fn update(&mut self, transform: &mut Transform, dt: f32) {
        // apply linear velocity
        transform.translation += self.linear_velocity * dt;

        // we have an angular velocity around the centre of mass, this needs to be converted to
        // relative body translation. This way we can properly update the rotation of the model

        let position_com = self.get_centre_of_mass_world(transform);
        let com_to_position = transform.translation - position_com;

        // total torque is equal to external applied torques + internal torque (precession)
        // T = T_external + omega x I * omega
        // T_external = 0 because it was applied in the collision response function
        // T = Ia = w x I * w
        // a = I^-1 (w x I * w)
        let orientation = Mat3::from_quat(transform.rotation);
        let inertia_tensor = orientation * self.shape.inertia_tensor() * orientation.transpose();
        let alpha = inertia_tensor.inverse()
            * (self
                .angular_velocity
                .cross(inertia_tensor * self.angular_velocity));
        self.angular_velocity += alpha * dt;

        // update orientation
        let d_angle = self.angular_velocity * dt;
        let angle = d_angle.length();
        let inv_angle = angle.recip();
        let dq = if inv_angle.is_finite() {
            Quat::from_axis_angle(d_angle * inv_angle, angle)
        } else {
            Quat::IDENTITY
        };
        transform.rotation = (dq * transform.rotation).normalize();

        // now get the new body position
        transform.translation = position_com + dq * com_to_position;
    }
}
