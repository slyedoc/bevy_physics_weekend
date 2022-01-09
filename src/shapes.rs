use bevy::prelude::*;

#[derive(Copy, Clone, Debug)]
pub enum PyhsicsShape {
    Sphere { radius: f32 },
}

impl Default for PyhsicsShape {
    fn default() -> PyhsicsShape {
        PyhsicsShape::Sphere { radius: 1.0 }
    }
}

impl PyhsicsShape {
    pub fn centre_of_mass(&self) -> Vec3 {
        match self {
            PyhsicsShape::Sphere { .. } => Vec3::ZERO,
        }
    }
}
