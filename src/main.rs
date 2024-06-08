use ggez::glam::*;
use ggez::graphics::{self, Canvas, Color, Rect};
use ggez::input::keyboard::KeyCode;
use ggez::winit::dpi::PhysicalSize;
use ggez::{event, GameError};
use ggez::{Context, GameResult};
use rand::Rng;
use std::time::Duration;
use std::{env, path};

struct MainState {
    window_size: PhysicalSize<u32>,
    floor_height: f32,
    opponent_position: Vec2,
    projectile: Option<Projectile>,
    gravity: Vec2,
    simulation_time_scale: f32,
}

struct Projectile {
    position: Vec2,
    velocity: Vec2,
    acceleration: Vec2,
    radius: f32,
}

impl Default for Projectile {
    fn default() -> Self {
        Self {
            position: Vec2::default(),
            velocity: Vec2::default(),
            acceleration: Vec2::default(),
            radius: 5.0,
        }
    }
}

impl Projectile {
    pub fn fire_from(position: Vec2) -> Self {
        // TODO: Convert from cartesian to polar, angle and force

        let mut rng = rand::thread_rng();
        let random_x: f32 = rng.gen_range(-10.0..=10.0);
        let random_y: f32 = rng.gen_range(-10.0..=30.0);

        Self {
            position,
            velocity: Vec2::new(-30.0 + random_x, -80.0 + random_y),
            acceleration: Vec2::new(0.0, 0.0),
            ..Default::default()
        }
    }

    pub fn step(&mut self, duration: Duration, gravity: Vec2, time_scale: f32) {
        let duration = duration.as_secs_f32() * time_scale;
        self.position += self.velocity * duration + 0.5 * self.acceleration * duration.powi(2);
        self.velocity += self.acceleration * duration;
        self.acceleration = gravity;
    }
}

impl MainState {
    fn new(ctx: &mut Context) -> GameResult<MainState> {
        ctx.gfx.add_font(
            "LiberationMono",
            graphics::FontData::from_path(ctx, "/LiberationMono-Regular.ttf")?,
        );

        let s = MainState {
            window_size: ctx.gfx.window().inner_size(),
            floor_height: 100.0,
            opponent_position: Vec2::default(),
            projectile: None,
            gravity: Vec2::new(0.0, 9.81),
            simulation_time_scale: 5.0,
        };
        Ok(s)
    }

    fn render_floor(&self, ctx: &mut Context, canvas: &mut Canvas) -> Result<(), GameError> {
        let window_size = ctx.gfx.window().inner_size();
        let circle = graphics::Mesh::new_rectangle(
            ctx,
            graphics::DrawMode::fill(),
            Rect::new(0.0, 0.0, window_size.width as f32, self.floor_height),
            Color::WHITE,
        )?;
        canvas.draw(
            &circle,
            Vec2::new(0.0, window_size.height as f32 - self.floor_height),
        );
        Ok(())
    }

    fn render_opponent(&self, ctx: &mut Context, canvas: &mut Canvas) -> Result<(), GameError> {
        let circle = graphics::Mesh::new_circle(
            ctx,
            graphics::DrawMode::fill(),
            Vec2::default(),
            20.0,
            0.2,
            Color::WHITE,
        )?;
        canvas.draw(&circle, self.opponent_position);
        Ok(())
    }

    fn render_projectile(&self, ctx: &mut Context, canvas: &mut Canvas) -> Result<(), GameError> {
        if let Some(projectile) = &self.projectile {
            let circle = graphics::Mesh::new_circle(
                ctx,
                graphics::DrawMode::fill(),
                Vec2::default(),
                5.0,
                0.2,
                Color::RED,
            )?;
            canvas.draw(&circle, projectile.position);
        }

        Ok(())
    }
}

impl event::EventHandler<GameError> for MainState {
    fn update(&mut self, ctx: &mut Context) -> GameResult {
        if ctx.keyboard.is_key_pressed(KeyCode::Return) {
            ctx.request_quit();
            return Ok(());
        }

        self.opponent_position = Vec2::new(
            (self.window_size.width as f32) * 0.9,
            self.window_size.height as f32 - self.floor_height,
        );

        // Update the projectile.
        if let Some(projectile) = self.projectile.as_mut() {
            projectile.step(ctx.time.delta(), self.gravity, self.simulation_time_scale);

            // If the projectile leaves the bounds or hits the ground, destroy and restart.
            if projectile.position.x <= projectile.radius
                || projectile.position.y <= projectile.radius
                || projectile.position.y
                    > (self.window_size.height as f32 - self.floor_height + projectile.radius)
            {
                self.projectile = None;
            }
        } else {
            self.projectile = Some(Projectile::fire_from(self.opponent_position));
        }

        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        let mut canvas = Canvas::from_frame(ctx, graphics::Color::from([0.1, 0.2, 0.3, 1.0]));

        // Text is drawn from the top-left corner.
        let dest_point = Vec2::new(10., 10.);
        canvas.draw(
            graphics::Text::new("Rawr, rawr never changes") // 🦖
                .set_font("LiberationMono")
                .set_scale(12.),
            dest_point,
        );

        self.render_floor(ctx, &mut canvas)?;
        self.render_projectile(ctx, &mut canvas)?;
        self.render_opponent(ctx, &mut canvas)?;

        canvas.finish(ctx)?;
        Ok(())
    }

    fn resize_event(
        &mut self,
        ctx: &mut Context,
        _width: f32,
        _height: f32,
    ) -> Result<(), GameError> {
        self.window_size = ctx.gfx.window().inner_size();
        Ok(())
    }
}

pub fn main() -> GameResult {
    let resource_dir = if let Ok(manifest_dir) = env::var("CARGO_MANIFEST_DIR") {
        let mut path = path::PathBuf::from(manifest_dir);
        path.push("resources");
        path
    } else {
        path::PathBuf::from("./resources")
    };

    let cb = ggez::ContextBuilder::new("super_simple", "ggez").add_resource_path(resource_dir);
    let (mut ctx, event_loop) = cb.build()?;
    let state = MainState::new(&mut ctx)?;
    event::run(ctx, event_loop, state)
}
