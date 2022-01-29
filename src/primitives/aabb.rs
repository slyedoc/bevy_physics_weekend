use bevy::{math::Vec3, prelude::Component};

// Axis-Aligned Bounding Box
#[derive(Default, Component)]
pub struct Aabb {
    pub mins: Vec3,
    pub maxs: Vec3,
}

impl Aabb {
    pub fn expand_by_point(&mut self, rhs: Vec3) {
        self.mins = Vec3::select(rhs.cmplt(self.mins), rhs, self.mins);
        self.maxs = Vec3::select(rhs.cmpgt(self.maxs), rhs, self.maxs);
    }
}
