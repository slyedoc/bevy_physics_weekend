use bevy::prelude::*;
use bevy_inspector_egui::Inspectable;


#[derive(Inspectable, Copy, Clone, Debug)]
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

    pub fn inertia_tensor(&self) -> Mat3 {
        match self {
            PyhsicsShape::Sphere { radius } => {
                Mat3::from_diagonal(Vec3::splat(2.0 * radius * radius / 5.0))
            }
        }
    }
}
