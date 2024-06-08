mod projectile;

use crate::projectile::Projectile;
use ggez::glam::*;
use ggez::graphics::{self, Canvas, Color, Drawable, Rect};
use ggez::winit::dpi::PhysicalSize;
use ggez::{conf, event, GameError};
use ggez::{Context, GameResult};
use std::time::Duration;
use std::{env, path};

const PHYSICS_FPS: u32 = 60;
pub(crate) const GAME_TIME_FACTOR: f32 = 10.0;

struct MainState {
    window_size: PhysicalSize<u32>,
    /// The opponent's position in the physical world.
    opponent_position: Vec2,
    projectile: Option<Projectile>,
    projectile_trajectory: Option<Vec<Vec2>>,
    gravity: Vec2,
    world_size: Vec2,
    world_to_screen: Mat3,
    screen_offset: Vec2,
    screen_scale: Vec2,
}

impl MainState {
    fn new(ctx: &mut Context) -> GameResult<MainState> {
        ctx.gfx.add_font(
            "LiberationMono",
            graphics::FontData::from_path(ctx, "/LiberationMono-Regular.ttf")?,
        );

        let s = MainState {
            window_size: ctx.gfx.window().inner_size(),
            opponent_position: Vec2::default(),
            projectile: None,
            projectile_trajectory: None,
            gravity: Vec2::new(0.0, -9.81),
            world_size: Vec2::new(2000.0, 2000.0),
            world_to_screen: Mat3::default(),
            screen_offset: Vec2::new(0.0, -100.0),
            screen_scale: Vec2::new(1.0, 1.0),
        };
        Ok(s)
    }

    fn floor_height(&self) -> f32 {
        -self.screen_offset.y
    }

    fn create_transformation_matrix(&self) -> Mat3 {
        let screen_width = self.window_size.width as f32;
        let screen_height = self.window_size.height as f32;

        let scale_x = screen_width / self.world_size.x * self.screen_scale.x;
        let scale_y = screen_height / self.world_size.y * self.screen_scale.y;

        // Create a scaling matrix with y-axis flipped and additional scale
        let scale_matrix = Mat3::from_scale(Vec2::new(scale_x, -scale_y));

        // Create a translation matrix to move the origin to the specified screen offset
        let translation_matrix = Mat3::from_translation(
            self.screen_offset + Vec2::new(0.0, screen_height * self.screen_scale.y),
        );

        // Combine the matrices (scaling first, then translation)
        translation_matrix * scale_matrix
    }

    fn render_floor(&self, ctx: &mut Context, canvas: &mut Canvas) -> Result<(), GameError> {
        let window_size = ctx.gfx.window().inner_size();
        let circle = graphics::Mesh::new_rectangle(
            ctx,
            graphics::DrawMode::fill(),
            Rect::new(0.0, 0.0, window_size.width as f32, self.floor_height()),
            Color::WHITE,
        )?;
        canvas.draw(
            &circle,
            Vec2::new(0.0, window_size.height as f32 - self.floor_height()),
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

        let pos = self
            .world_to_screen
            .transform_point2(self.opponent_position);
        canvas.draw(&circle, pos);
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

            let pos = self.world_to_screen.transform_point2(projectile.position);
            canvas.draw(&circle, pos);
        }

        Ok(())
    }

    fn render_projectile_trajectory(
        &self,
        ctx: &mut Context,
        canvas: &mut Canvas,
    ) -> Result<(), GameError> {
        if let Some(trajectory) = &self.projectile_trajectory {
            let circle = graphics::Mesh::new_circle(
                ctx,
                graphics::DrawMode::fill(),
                Vec2::default(),
                1.0,
                0.2,
                Color::new(1.0, 0.0, 0.0, 0.125),
            )?;

            for &pos in trajectory {
                let pos = self.world_to_screen.transform_point2(pos);
                canvas.draw(&circle, pos);
            }
        }

        Ok(())
    }

    fn projectile_in_bounds(&self, projectile: Vec2, radius: f32) -> bool {
        let pos = self.world_to_screen.transform_point2(projectile);

        !(pos.x <= radius
            || pos.y <= radius
            || pos.y > (self.window_size.height as f32 - self.floor_height() + radius))
    }

    fn update_transformations(&mut self) {
        self.world_to_screen = self.create_transformation_matrix();
    }
}

impl event::EventHandler<GameError> for MainState {
    fn update(&mut self, ctx: &mut Context) -> GameResult {
        while ctx.time.check_update_time((PHYSICS_FPS as f32) as u32) {
            let time_delta =
                Duration::from_secs_f32((PHYSICS_FPS as f32).recip() * GAME_TIME_FACTOR);

            self.opponent_position = Vec2::new(self.world_size.x * 0.9, 0.0);

            // Update the projectile.
            if let Some(projectile) = self.projectile.as_mut() {
                projectile.step(time_delta, self.gravity);
            } else {
                let projectile = Projectile::fire_from(self.opponent_position);

                let trajectory: Vec<_> = projectile
                    .simulate(
                        self.gravity,
                        Duration::from_secs_f32((PHYSICS_FPS as f32).recip() * GAME_TIME_FACTOR),
                        Duration::from_secs_f32(0.1),
                    )
                    .take_while(|&pos| self.projectile_in_bounds(pos, 0.0))
                    .collect();

                self.projectile = Some(projectile);
                self.projectile_trajectory = Some(trajectory);
            }

            // Reset the projectile if out of bounds.
            if let Some(projectile) = &self.projectile {
                if !self.projectile_in_bounds(projectile.position, projectile.radius) {
                    self.projectile = None;
                    self.projectile_trajectory = None;
                }
            }
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

        // Position at top-right
        if let Some(projectile) = &self.projectile {
            let mut text = graphics::Text::new(format!(
                "{:.2}, {:.2}",
                projectile.position.x, projectile.position.y
            ));
            text.set_font("LiberationMono").set_scale(12.);
            let text_dims_pos = text.dimensions(ctx).unwrap();
            let dest_point = Vec2::new(self.window_size.width as f32 - text_dims_pos.w - 10.0, 10.);
            canvas.draw(&text, dest_point);

            let mut text = graphics::Text::new(format!("{:.2} m/s²", projectile.velocity()));
            text.set_font("LiberationMono").set_scale(12.);
            let text_dims = text.dimensions(ctx).unwrap();
            let dest_point = Vec2::new(
                self.window_size.width as f32 - text_dims.w - 10.0,
                10. + text_dims_pos.y + text_dims_pos.h + 2.,
            );
            canvas.draw(&text, dest_point);
        }

        self.render_floor(ctx, &mut canvas)?;
        self.render_projectile_trajectory(ctx, &mut canvas)?;
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
        self.update_transformations();
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

    let cb = ggez::ContextBuilder::new("projectile_estimation", "me")
        .window_setup(conf::WindowSetup::default().title("Projectile Estimation"))
        .window_mode(conf::WindowMode::default().dimensions(640.0, 480.0))
        .add_resource_path(resource_dir);
    let (mut ctx, event_loop) = cb.build()?;
    let state = MainState::new(&mut ctx)?;
    event::run(ctx, event_loop, state)
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn equal_scaling_works() {
        let mut s = MainState {
            window_size: PhysicalSize::new(640, 480),
            opponent_position: Vec2::default(),
            projectile: None,
            projectile_trajectory: None,
            gravity: Vec2::new(0.0, 9.81),
            world_size: Vec2::new(640.0, 480.0),
            world_to_screen: Mat3::default(),
            screen_offset: Vec2::default(),
            screen_scale: Vec2::ONE,
        };

        s.update_transformations();

        let bottom_left = s.world_to_screen.transform_point2(Vec2::default());
        let top_of_bottom_left = s.world_to_screen.transform_point2(Vec2::new(0.0, 100.0));
        let top_right_of_bottom_left = s.world_to_screen.transform_point2(Vec2::new(100.0, 100.));

        assert_eq!(bottom_left, Vec2::new(0.0, 480.0));
        assert_eq!(top_of_bottom_left, Vec2::new(0.0, 380.0));
        assert_eq!(top_right_of_bottom_left, Vec2::new(100.0, 380.0));
    }

    #[test]
    fn different_scaling_works() {
        let mut s = MainState {
            window_size: PhysicalSize::new(640, 480),
            opponent_position: Vec2::default(),
            projectile: None,
            projectile_trajectory: None,
            gravity: Vec2::new(0.0, 9.81),
            world_size: Vec2::new(640.0 * 2.0, 480.0 * 2.0), // twice the size of the screen
            world_to_screen: Mat3::default(),
            screen_offset: Vec2::new(1000.0, 2000.0),
            screen_scale: Vec2::ONE,
        };

        s.update_transformations();

        let bottom_left = s.world_to_screen.transform_point2(Vec2::default());
        let top_of_bottom_left = s.world_to_screen.transform_point2(Vec2::new(0.0, 100.0));
        let top_right_of_bottom_left = s.world_to_screen.transform_point2(Vec2::new(100.0, 100.));

        assert_eq!(bottom_left, Vec2::new(1000.0 + 0.0, 2000.0 + 480.0));
        assert_eq!(
            top_of_bottom_left,
            Vec2::new(1000.0 + 0.0, 2000.0 + 480.0 - 50.0)
        );
        assert_eq!(
            top_right_of_bottom_left,
            Vec2::new(1000.0 + 50.0, 2000.0 + 480.0 - 50.0)
        );
    }
}
