use crate::miniquad::{BlendFactor, BlendState, BlendValue, Equation};
use macroquad::{audio::*, prelude::*, rand::*};
use rapier2d::prelude::*;
use std::sync::RwLock;

const PLAYER_ACCEL: f32 = 3000000.0;

fn frand() -> f32 {
    rand() as f32 / u32::MAX as f32
}

pub struct MyContact(ColliderHandle, ColliderHandle);

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Tile {
    Wall,
    Floor { has_battery: bool },
}

#[derive(Debug)]
pub struct Map {
    tiles: Vec<Vec<Tile>>,
}

impl Map {
    fn new() -> Self {
        let mut tiles = vec![vec![Tile::Wall; 100]; 100];
        for y in 1..99 {
            for x in 1..99 {
                let roll = gen_range(1, 100);
                if roll > 55 {
                    tiles[y][x] = Tile::Floor { has_battery: false };
                }
            }
        }

        for _ in 0..15 {
            let mut new_tiles = tiles.clone();
            for y in 1..99 {
                for x in 1..99 {
                    let mut neighbors = 0;
                    for dx in -1_isize..=1 {
                        for dy in -1_isize..=1 {
                            if (dx != 0 || dy != 0)
                                && tiles[(y as isize + dy) as usize][(x as isize + dx) as usize]
                                    == Tile::Wall
                            {
                                neighbors += 1;
                            }
                        }
                    }
                    if neighbors > 4 || neighbors == 0 {
                        new_tiles[y][x] = Tile::Wall;
                    } else {
                        new_tiles[y][x] = Tile::Floor {
                            has_battery: frand() > 0.99,
                        };
                    }
                }
            }
            tiles = new_tiles;
        }
        Map { tiles }
    }
}

pub struct Physics {
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    joint_set: JointSet,
    ccd_solver: CCDSolver,
    last_contacts: Contacts,
}

struct Contacts {
    contacts: RwLock<Vec<IntersectionEvent>>,
}

impl EventHandler for Contacts {
    fn handle_intersection_event(&self, event: IntersectionEvent) {
        let mut intersections = self.contacts.write().unwrap();
        intersections.push(event);
    }
    fn handle_contact_event(&self, _event: ContactEvent, _contact_pair: &ContactPair) {}
}

impl Physics {
    pub fn new() -> Self {
        Self {
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            integration_parameters: IntegrationParameters::default(),
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            joint_set: JointSet::new(),
            ccd_solver: CCDSolver::new(),
            last_contacts: Contacts {
                contacts: RwLock::new(vec![]),
            },
        }
    }

    pub fn step(&mut self) {
        let gravity = vector![0.0, 0.0];
        let physics_hooks = ();
        self.last_contacts.contacts.write().unwrap().clear();
        self.physics_pipeline.step(
            &gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.joint_set,
            &mut self.ccd_solver,
            &physics_hooks,
            &self.last_contacts,
        );
    }
}

enum GameState {
    StartScreen,
    Playing,
    Death { death_timer: f32 },
    EndScreen,
}

#[macroquad::main("test")]
async fn main() {
    info!("Setting up RNG");
    srand(crate::miniquad::date::now() as u64);
    info!("Loading audio");
    let snd_heart_beat = load_sound("heartbeat.wav").await.unwrap();
    let snd_death1 = load_sound("death1.wav").await.unwrap();

    info!("Loading shaders");
    let occluders_texture = render_target(256, 256);
    let shadow_map = render_target(256, 1);
    let shadow_map_shader =
        load_material(DEFAULT_VERTEX_SHADER, SHADOW_MAP_FRAG, Default::default()).unwrap();
    let shadow_render_shader = load_material(
        DEFAULT_VERTEX_SHADER,
        SHADOW_RENDER_FRAG_SHADER,
        MaterialParams {
            uniforms: vec![("Color".to_string(), UniformType::Float4)],
            pipeline_params: PipelineParams {
                color_blend: Some(BlendState::new(
                    Equation::Add,
                    BlendFactor::Value(BlendValue::SourceAlpha),
                    BlendFactor::OneMinusValue(BlendValue::SourceAlpha),
                )),
                ..Default::default()
            },
            ..Default::default()
        },
    )
    .unwrap();
    loop {
        info!("Starting new game");

        info!("Setting up physics");
        let mut physics = Physics::new();

        info!("Generating map");
        let map = Map::new();
        let mut player_start = None;
        for y in 0..100 {
            let mut conseq = 0;
            for x in 0..101 {
                if x < 100 && map.tiles[y][x] == Tile::Wall {
                    conseq += 1;
                } else if conseq > 0 {
                    let collider = ColliderBuilder::cuboid(35.0 * conseq as f32, 35.0)
                        .translation(vector![(x * 2 - conseq) as f32 * 35.0, y as f32 * 70.0])
                        .build();
                    physics.collider_set.insert(collider);
                    conseq = 0;
                }
                if x < 100 && conseq == 0 {
                    if let Tile::Floor { has_battery } = map.tiles[y][x] {
                        if has_battery {
                            let collider = ColliderBuilder::round_cuboid(8.0, 3.0, 1.0)
                                .translation(vector![x as f32 * 70.0 + 35.0, y as f32 * 70.0])
                                .sensor(true)
                                .active_events(ActiveEvents::INTERSECTION_EVENTS)
                                .build();
                            physics.collider_set.insert(collider);
                        }
                    }
                    if player_start.is_none() {
                        player_start = Some(vector![x as f32 * 70.0 + 35.0, y as f32 * 70.0]);
                    }
                }
            }
        }

        info!("Adding player");
        let player_body = RigidBodyBuilder::new_dynamic()
            .translation(player_start.unwrap())
            .linear_damping(8.0)
            .lock_rotations()
            .build();
        let collider = ColliderBuilder::cuboid(20.0, 30.0).friction(0.0).build();
        let player_body = physics.rigid_body_set.insert(player_body);
        let player_collider_handle = physics.collider_set.insert_with_parent(
            collider,
            player_body,
            &mut physics.rigid_body_set,
        );

        let mut phys_time = 0.0;

        // Setup audio
        play_sound(
            snd_heart_beat,
            PlaySoundParams {
                looped: true,
                volume: 0.0,
                ..Default::default()
            },
        );
        let mut last_volume_change = 0.0;
        let mut light_energy = 1.0;
        let mut state = GameState::StartScreen;

        let mut stats_batteries = 0;
        let mut stats_lifetime = 0.0;

        let subject_name = format!("#{}", rand() % 97861);
        let mut random_eye: Option<Vec2> = None;

        loop {
            let delta = get_frame_time();
            let player_body = &mut physics.rigid_body_set[player_body];
            let bp = player_body.translation();
            let player_pos = vec2(bp.x, bp.y);

            // Render occluders
            set_camera(&Camera2D {
                zoom: vec2(1.0 / 256.0, 1.0 / 256.0),
                target: player_pos,
                render_target: Some(occluders_texture),
                ..Default::default()
            });
            clear_background(BLANK);
            for (_, collider) in physics.collider_set.iter() {
                // Skip player for lighting
                if collider.parent().is_some() {
                    continue;
                }

                if let Some(cuboid) = collider.shape().as_cuboid() {
                    let center = Vec2::from_slice_unaligned(collider.translation().as_slice());
                    let extent = cuboid.half_extents;
                    let extent = vec2(extent.x, extent.y);
                    let rot = Mat2::from_angle(collider.rotation().angle());
                    let tl = rot.mul_vec2(-extent) + center;
                    let tr = rot.mul_vec2(vec2(1.0, -1.0) * extent) + center;
                    let br = rot.mul_vec2(extent) + center;
                    let bl = rot.mul_vec2(vec2(-1.0, 1.0) * extent) + center;
                    draw_triangle(tl, tr, br, WHITE);
                    draw_triangle(tl, br, bl, WHITE);
                }
            }

            // Render shadow lookup texture
            set_camera(&Camera2D {
                render_target: Some(shadow_map),
                ..Camera2D::from_display_rect(Rect::new(0.0, 0.0, 256.0, 256.0))
            });
            gl_use_material(shadow_map_shader);

            draw_texture(occluders_texture.texture, 0.0, 0.0, WHITE);

            gl_use_default_material();

            // Render game
            let mut camera =
                Camera2D::from_display_rect(Rect::new(0.0, 0.0, screen_width(), screen_height()));
            camera.target = player_pos;
            set_camera(&camera);

            clear_background(BLACK);

            fn draw_entity(collider: &Collider, player_pos: Vec2) {
                let center = Vec2::from_slice_unaligned(collider.translation().as_slice());
                let dst = 20.0_f32.max(center.distance_squared(player_pos));
                let alpha = 1.0_f32.min(20000.0 / dst);
                if alpha < 0.01 {
                    return;
                }
                let rot = Mat2::from_angle(collider.rotation().angle());

                if let Some(cuboid) = collider.shape().as_cuboid() {
                    let color = if collider.parent().is_none() {
                        Color::new(0.1, 0.1, 0.1, alpha)
                    } else {
                        Color::new(0.3, 0.0, 0.0, alpha)
                    };

                    let extent = cuboid.half_extents;
                    let extent = vec2(extent.x, extent.y);
                    let tl = rot.mul_vec2(-extent) + center;
                    let tr = rot.mul_vec2(vec2(1.0, -1.0) * extent) + center;
                    let br = rot.mul_vec2(extent) + center;
                    let bl = rot.mul_vec2(vec2(-1.0, 1.0) * extent) + center;
                    draw_triangle(tl, tr, br, color);
                    draw_triangle(tl, br, bl, color);
                }

                if let Some(round_cuboid) = collider.shape().as_round_cuboid() {
                    let color = Color::new(0.1, 0.7, 0.1, alpha);
                    let extent = round_cuboid.base_shape.half_extents;
                    let extent = vec2(extent.x, extent.y);
                    let tl = rot.mul_vec2(-extent) + center;
                    let tr = rot.mul_vec2(vec2(1.0, -1.0) * extent) + center;
                    let br = rot.mul_vec2(extent) + center;
                    let bl = rot.mul_vec2(vec2(-1.0, 1.0) * extent) + center;
                    let left = rot.mul_vec2(vec2(-1.0, 0.0) * extent) + center;
                    let right = rot.mul_vec2(vec2(1.0, 0.0) * extent) + center;
                    draw_triangle(tl, tr, br, color);
                    draw_triangle(tl, br, bl, color);
                    draw_circle(left.x, left.y, extent.y / 2.0, color);
                    draw_circle(right.x, right.y, extent.y / 2.0, color);
                }
            }

            for (_, collider) in physics.collider_set.iter() {
                draw_entity(collider, player_pos);
            }

            if let Some(eye) = random_eye {
                if eye.distance(player_pos) < 200.0 || eye.distance(player_pos) > 300.0 {
                    random_eye = None;
                } else {
                    let iris = eye + (player_pos - eye).normalize() * 4.0;
                    draw_circle(eye.x, eye.y, 10.0, Color::new(0.15, 0.0, 0.0, 1.0));
                    draw_circle(iris.x, iris.y, 6.0, Color::new(0.0, 0.0, 0.0, 1.0));
                }
            } else if frand() < delta / 5.0 {
                random_eye = Some(
                    player_pos
                        + 250.0
                            * Mat2::from_angle(frand() * std::f32::consts::PI * 2.0)
                                .mul_vec2(Vec2::X),
                );
            }

            // Render light
            let actual_light = if frand() < 2.0 * light_energy {
                light_energy
            } else {
                light_energy * frand()
            };
            shadow_render_shader.set_uniform(
                "Color",
                vec4(actual_light * 0.8, actual_light * 0.9, actual_light, 1.0),
            );
            gl_use_material(shadow_render_shader);

            draw_texture_ex(
                shadow_map.texture,
                bp.x - 256.0,
                bp.y - 256.0,
                WHITE,
                DrawTextureParams {
                    dest_size: Some(vec2(512.0, 512.0)),
                    ..Default::default()
                },
            );

            gl_use_default_material();

            // Redraw player
            match state {
                GameState::Playing => {
                    draw_entity(&physics.collider_set[player_collider_handle], player_pos);
                }
                GameState::Death { death_timer } => {
                    draw_entity(&physics.collider_set[player_collider_handle], player_pos);
                    if death_timer < 0.2 {
                        clear_background(WHITE);
                    }
                }
                GameState::StartScreen | GameState::EndScreen { .. } => {}
            }

            set_default_camera();

            match state {
                GameState::EndScreen => {
                    let rect = Rect::new(
                        screen_width() / 2.0 - 200.0,
                        screen_height() / 2.0 - 150.0,
                        400.0,
                        300.0,
                    );
                    draw_rectangle(rect.x, rect.y, rect.w, rect.h, BROWN);
                    draw_text(
                        &format!("Survived: {:.1} seconds", stats_lifetime),
                        rect.x + 10.0,
                        rect.y + 40.0,
                        30.0,
                        DARKBROWN,
                    );
                    draw_text(
                        &format!("Batteries collected: {}", stats_batteries),
                        rect.x + 10.0,
                        rect.y + 70.0,
                        30.0,
                        DARKBROWN,
                    );
                    draw_text(
                        "Press Space to retry",
                        rect.x + 10.0,
                        rect.bottom() - 10.0,
                        30.0,
                        DARKPURPLE,
                    );
                }
                GameState::StartScreen => {
                    let rect = Rect::new(
                        screen_width() / 2.0 - 300.0,
                        screen_height() / 2.0 - 150.0,
                        600.0,
                        300.0,
                    );
                    draw_rectangle(rect.x, rect.y, rect.w, rect.h, BROWN);
                    for (i, s) in [
                        &format!("Welcome, Test-Subject {}", subject_name),
                        "Your test today involves finding the exit",
                        "in the maze prepared for you.",
                        "The light we provide you needs a steady",
                        "amount of batteries. Be sure to locate them",
                        "while searching.",
                        "It must not run out of juice.",
                    ]
                    .iter()
                    .enumerate()
                    {
                        draw_text(
                            s,
                            rect.x + 10.0,
                            rect.y + 40.0 + i as f32 * 30.0,
                            30.0,
                            DARKBROWN,
                        );
                    }
                    draw_text(
                        "Press Space to start",
                        rect.x + 10.0,
                        rect.bottom() - 10.0,
                        30.0,
                        DARKPURPLE,
                    );
                }
                GameState::Playing | GameState::Death { .. } => {}
            }

            #[cfg(debug_assertions)]
            draw_texture(occluders_texture.texture, 0., 30., WHITE);
            #[cfg(debug_assertions)]
            draw_texture_ex(
                shadow_map.texture,
                0.,
                300.0,
                WHITE,
                DrawTextureParams {
                    dest_size: Some(vec2(256.0, 30.0)),
                    ..Default::default()
                },
            );

            // GAME LOGIC
            match state {
                GameState::Playing => {
                    let mut move_force = Vector::new(0.0, 0.0);
                    if is_key_down(KeyCode::Left) {
                        move_force += Vector::new(-PLAYER_ACCEL, 0.0);
                    }
                    if is_key_down(KeyCode::Right) {
                        move_force += Vector::new(PLAYER_ACCEL, 0.0);
                    }
                    if is_key_down(KeyCode::Up) {
                        move_force += Vector::new(0.0, -PLAYER_ACCEL);
                    }
                    if is_key_down(KeyCode::Down) {
                        move_force += Vector::new(0.0, PLAYER_ACCEL);
                    }
                    if move_force.magnitude_squared() > 0.0 {
                        player_body.apply_force(move_force, true);
                    }
                }
                GameState::EndScreen => {
                    if is_key_pressed(KeyCode::Space) {
                        break;
                    }
                }
                GameState::StartScreen => {
                    if is_key_pressed(KeyCode::Space) {
                        state = GameState::Playing;
                    }
                }
                GameState::Death { .. } => {}
            }

            phys_time += delta;
            while phys_time >= 1.0 / 60.0 {
                physics.step();
                phys_time -= 1.0 / 60.0;
            }

            for IntersectionEvent {
                mut collider1,
                mut collider2,
                intersecting: _,
            } in physics.last_contacts.contacts.read().unwrap().iter()
            {
                if collider2 == player_collider_handle {
                    std::mem::swap(&mut collider1, &mut collider2);
                }
                if collider1 == player_collider_handle {
                    // Check for battery
                    if physics.collider_set[collider2].shape().shape_type()
                        == ShapeType::RoundCuboid
                    {
                        light_energy = 1.0_f32.min(light_energy + 0.5);
                        stats_batteries += 1;
                        physics.collider_set.remove(
                            collider2,
                            &mut physics.island_manager,
                            &mut physics.rigid_body_set,
                            false,
                        );
                    }
                }
            }

            match state {
                GameState::Playing => {
                    light_energy = (light_energy - delta / 20.0).max(0.0);
                    stats_lifetime += delta;
                    last_volume_change += delta;
                    // setting sound to often seems to kill the mechanism
                    if last_volume_change > 0.3 {
                        set_sound_volume(snd_heart_beat, 1.0 - light_energy);
                        last_volume_change = 0.0;
                    }
                    if light_energy <= 0.0 {
                        stop_sound(snd_heart_beat);
                        state = GameState::Death { death_timer: 3.0 };
                    }
                }
                GameState::Death {
                    ref mut death_timer,
                } => {
                    if *death_timer > 0.0 {
                        *death_timer -= delta;
                        if *death_timer <= 0.0 {
                            play_sound(snd_death1, PlaySoundParams::default());
                            state = GameState::EndScreen;
                        }
                    }
                }
                GameState::StartScreen | GameState::EndScreen => {}
            }

            next_frame().await
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test() {
        let map = Map::new();
    }
}

// Stolen from old libgdx tutorial
const SHADOW_MAP_FRAG: &'static str = r#"#version 100
#define PI 3.14
#define resolution vec2(256, 256)
precision lowp float;

//inputs from vertex shader
varying vec2 uv;

//uniform values
uniform sampler2D Texture;

//alpha threshold for our occlusion map
const float THRESHOLD = 0.75;

void main(void) {
  float distance = 1.0;

  for (lowp float y=0.0; y<resolution.y; y+=1.0) {
        //rectangular to polar filter
        vec2 norm = vec2(uv.s, y/resolution.y) * 2.0 - 1.0;
        float theta = PI*1.5 + norm.x * PI; 
        float r = (1.0 + norm.y) * 0.5;

        //coord which we will sample from occlude map
        vec2 coord = vec2(-r * sin(theta), -r * cos(theta))/2.0 + 0.5;

        //sample the occlusion map
        vec4 data = texture2D(Texture, coord);

        //the current distance is how far from the top we've come
        float dst = y/resolution.y;

        //if we've hit an opaque fragment (occluder), then get new distance
        //if the new distance is below the current, then we'll use that for our ray
        float caster = data.a;
        if (caster > THRESHOLD) {
            distance = min(distance, dst);
            //NOTE: we could probably use "break" or "return" here
        }
  } 
  gl_FragColor = vec4(vec3(distance), 1.0);
}
"#;

const DEFAULT_VERTEX_SHADER: &'static str = r#"#version 100
attribute vec3 position;
attribute vec2 texcoord;
varying lowp vec2 uv;
uniform mat4 Model;
uniform mat4 Projection;
void main() {
    gl_Position = Projection * Model * vec4(position, 1);
    uv = texcoord;
}
"#;

const SHADOW_RENDER_FRAG_SHADER: &'static str = r#"#version 100
#define PI 3.14
#define resolution vec2(256, 256)
precision lowp float;

//inputs from vertex shader
varying vec2 uv;
//varying LOWP vec4 vColor;

//uniform values
uniform sampler2D Texture;
uniform lowp vec4 Color;
//uniform vec2 resolution;

//sample from the 1D distance map
float sample(vec2 coord, float r) {
    return step(r, texture2D(Texture, coord).r);
}

void main(void) {
    //rectangular to polar
    vec2 norm = uv.st * 2.0 - 1.0;
    float theta = atan(-norm.y, norm.x);
    float r = length(norm); 
    float coord = (theta + PI) / (2.0*PI);

    //the tex coord to sample our 1D lookup texture 
    //always 0.0 on y axis
    vec2 tc = vec2(coord, 0.0);

    //the center tex coord, which gives us hard shadows
    float center = sample(tc, r);        

    //we multiply the blur amount by our distance from center
    //this leads to more blurriness as the shadow "fades away"
    float blur = (1./resolution.x)  * smoothstep(0., 1., r); 

    //now we use a simple gaussian blur
    float sum = 0.0;

    sum += sample(vec2(tc.x - 4.0*blur, tc.y), r) * 0.05;
    sum += sample(vec2(tc.x - 3.0*blur, tc.y), r) * 0.09;
    sum += sample(vec2(tc.x - 2.0*blur, tc.y), r) * 0.12;
    sum += sample(vec2(tc.x - 1.0*blur, tc.y), r) * 0.15;

    sum += center * 0.16;

    sum += sample(vec2(tc.x + 1.0*blur, tc.y), r) * 0.15;
    sum += sample(vec2(tc.x + 2.0*blur, tc.y), r) * 0.12;
    sum += sample(vec2(tc.x + 3.0*blur, tc.y), r) * 0.09;
    sum += sample(vec2(tc.x + 4.0*blur, tc.y), r) * 0.05;

    //sum of 1.0 -> in light, 0.0 -> in shadow

    //multiply the summed amount by our distance, which gives us a radial falloff
    //then multiply by vertex (light) color  
    gl_FragColor = Color * vec4(vec3(1.0), sum * smoothstep(1.0, 0.0, r));
}
"#;
