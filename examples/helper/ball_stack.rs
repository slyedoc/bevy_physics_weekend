use bevy::prelude::*;
use bevy_inspector_egui::Inspectable;

#[derive(Inspectable)]
pub struct BallStackConfig {
    pub count: usize,
    pub base_size: usize,
    pub grid_offset: f32,
    pub start_height: f32,
    pub ball_radius: f32,
    pub ball_sectors: usize,
    pub ball_stacks: usize,

    #[inspectable(ignore)]
    pub ball_material: Handle<StandardMaterial>,
}

impl FromWorld for BallStackConfig {
    fn from_world(world: &mut World) -> Self {

        let world = world.cell();
        let mut materials = world
            .get_resource_mut::<Assets<StandardMaterial>>()
            .unwrap();

        let asset_server = world.get_resource_mut::<AssetServer>().unwrap();

        Self {
            count: 1000,
            base_size: 10,
            grid_offset: 3.0,
            ball_radius: 1.0,
            ball_sectors: 8,
            ball_stacks: 8,

            ball_material: materials.add(StandardMaterial {
                base_color_texture: Some(asset_server.load("checker_red.png")),
                ..Default::default()
            }),
            start_height: 5.0,
        }
    }
}


#[allow(dead_code)]
pub fn ball_count_system(
    input: Res<Input<KeyCode>>,
    mut config: ResMut<BallStackConfig>,
) {
    if input.just_pressed(KeyCode::Equals) {
        config.count += 1000;
    }

    if input.just_pressed(KeyCode::Minus) {
        config.count = (config.count - 1000).clamp(0, 50000);
    }
}
