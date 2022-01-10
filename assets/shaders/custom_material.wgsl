#import bevy_pbr::mesh_view_bind_group
#import bevy_pbr::mesh_struct

[[group(1), binding(0)]]
var<uniform> mesh: Mesh;

struct Vertex {
    [[location(0)]] position: vec3<f32>;
    [[location(1)]] normal: vec3<f32>;
    [[location(2)]] uv: vec2<f32>;
};

struct VertexOutput {
    [[builtin(position)]] clip_position: vec4<f32>;
    [[location(0)]] uv: vec2<f32>;
};

struct CustomMaterial {
    color1: vec4<f32>;
    color2: vec4<f32>;
};
[[group(1), binding(0)]]
var<uniform> material: CustomMaterial;

[[stage(vertex)]]
fn vertex(vertex: Vertex) -> VertexOutput {
    let world_position = mesh.model * vec4<f32>(vertex.position, 1.0);

    var out: VertexOutput;
    out.clip_position = view.view_proj * world_position;
    out.uv = vertex.uv;
    return out;
}

// fn inverse_lerp(a: f32, b: f32, v: f32) -> f32 {
//     return (v-a) / (b-a);
// }

[[stage(fragment)]]
fn fragment( in: VertexOutput ) -> [[location(0)]] vec4<f32> {

    // fract uv to draw multiple tiles
	let uv = fract(in.uv - 0.5);

    //uv = fract(uv);


    // determine whether uv position is even or odd and store it in a variable
    //let mod = floor(uv.x) + floor(uv.y) % 2.0;
    // background color
    //let color = material.color1 * blackOrWhite + material.color2 * (1.0 - blackOrWhite);
    // Output to screen    
    //let fragColor = vec4<f32>(vec3<f32>(step(uv.x * uv.y, 0.)), 1.);
    return vec4<f32>(uv.x, 0.0, 0.0, 1.0);
}
