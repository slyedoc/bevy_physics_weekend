use crate::shapes::*;
use bevy::prelude::*;

#[derive(Component, Copy, Clone, Debug)]
pub struct Body {
    pub linear_velocity: Vec3,
    pub inv_mass: f32,
    pub shape: PyhsicsShape,
}

impl Default for Body {
    fn default() -> Self {
        Self {
            linear_velocity: Vec3::default(),
            inv_mass: 1.0,
            shape: PyhsicsShape::default(),
        }
    }
}

impl Body {
    // pub fn centre_of_mass_world_space(&self) -> Vec3 {
    //     let com = self.shape.centre_of_mass();
    //     self.position + self.orientation * com
    // }
    // pub fn centre_of_mass_model_space(&self) -> Vec3 {
    //     self.shape.centre_of_mass()
    // }
    // pub fn world_space_to_body_space(&self, world_point: Vec3) -> Vec3 {
    //     let tmp = world_point - self.centre_of_mass_world_space();
    //     let inv_orientation = self.orientation.conjugate();
    //     let body_space = inv_orientation * tmp;
    //     body_space
    // }
    // pub fn body_space_to_world_space(&self, body_point: Vec3) -> Vec3 {
    //     let world_point = self.centre_of_mass_world_space() + self.orientation * body_point;
    //     world_point
    // }
    pub fn apply_impulse_linear(&mut self, impulse: Vec3) {
        if self.inv_mass != 0.0 {
            // p = mv
            // dp = m dv = J
            // => dv = J / m
            self.linear_velocity += impulse * self.inv_mass;
        }
    }
    // pub fn integrate(&mut self, delta_seconds: f32) {
    //     self.position += self.linear_velocity * delta_seconds;
    // }

}
