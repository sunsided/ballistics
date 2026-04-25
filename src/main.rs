mod projectile;
mod tracker;

use crate::projectile::{Projectile, Wind};
use crate::tracker::{ImpactPrediction, Tracker};
use ggez::glam::*;
use ggez::graphics::{self, Canvas, Color, Drawable, Rect};
use ggez::winit::dpi::PhysicalSize;
use ggez::{conf, event, GameError};
use ggez::{Context, GameResult};
use rand::RngExt;
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
    tracker: Option<Tracker>,
    wind: Wind,
}

impl MainState {
    fn new(ctx: &mut Context) -> GameResult<MainState> {
        ctx.gfx.add_font(
            "LiberationMono",
            graphics::FontData::from_path(ctx, "/LiberationMono-Regular.ttf")?,
        );

        let dt = (PHYSICS_FPS as f32).recip() * GAME_TIME_FACTOR;
        let tracker = Some(Tracker::new(dt, 1.0, 25.0));
        let wind = Wind::new(5.0, 0.5, 2.0, 3.0);

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
            tracker,
            wind,
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
                Color::new(0.25, 0.35, 0.65, 0.5),
            )?;

            for &pos in trajectory {
                let pos = self.world_to_screen.transform_point2(pos);
                canvas.draw(&circle, pos);
            }
        }

        Ok(())
    }

    fn render_filter_trajectory(
        &self,
        ctx: &mut Context,
        canvas: &mut Canvas,
    ) -> Result<(), GameError> {
        if let Some(tracker) = &self.tracker {
            let sim_dt = (PHYSICS_FPS as f32).recip() * GAME_TIME_FACTOR;
            if let Some(trajectory) = tracker.predicted_trajectory(sim_dt, 0.1, 5000) {
                let circle = graphics::Mesh::new_circle(
                    ctx,
                    graphics::DrawMode::fill(),
                    Vec2::default(),
                    1.0,
                    0.2,
                    Color::new(0.3, 1.0, 0.3, 0.2),
                )?;

                for pos in trajectory {
                    let pos = self.world_to_screen.transform_point2(pos);
                    canvas.draw(&circle, pos);
                }
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

    fn render_tracker_estimate(
        &self,
        ctx: &mut Context,
        canvas: &mut Canvas,
    ) -> Result<(), GameError> {
        if let Some(tracker) = &self.tracker {
            if tracker.is_initialized() {
                let pos = tracker.estimated_position();
                let screen_pos = self.world_to_screen.transform_point2(pos);

                let circle = graphics::Mesh::new_circle(
                    ctx,
                    graphics::DrawMode::fill(),
                    Vec2::default(),
                    5.0,
                    0.2,
                    Color::CYAN,
                )?;
                canvas.draw(&circle, screen_pos);

                let (sigma_x, sigma_y, correlation) = tracker.position_covariance_1sigma();
                if sigma_x > 0.1 && sigma_y > 0.1 {
                    let ellipse = covariance_ellipse(sigma_x, sigma_y, correlation, 12);
                    let ellipse_mesh = graphics::Mesh::new_line(ctx, &ellipse, 2.0, Color::CYAN)?;
                    canvas.draw(&ellipse_mesh, screen_pos);
                }
            }
        }
        Ok(())
    }

    fn render_predicted_impact(
        &self,
        ctx: &mut Context,
        canvas: &mut Canvas,
    ) -> Result<(), GameError> {
        if let Some(tracker) = &self.tracker {
            if let Some(prediction) =
                tracker.predict_impact((PHYSICS_FPS as f32).recip() * GAME_TIME_FACTOR, 200, 5000)
            {
                self.draw_impact_zone(ctx, canvas, &prediction)?;
            }
        }
        Ok(())
    }

    fn render_trajectory_legend(
        &self,
        _ctx: &mut Context,
        canvas: &mut Canvas,
    ) -> Result<(), GameError> {
        let mut firing_text = graphics::Text::new(
            graphics::TextFragment::new("blue = firing trajectory (gravity only)")
                .color(Color::new(0.35, 0.5, 0.9, 0.9)),
        );
        firing_text.set_font("LiberationMono").set_scale(11.);
        canvas.draw(&firing_text, Vec2::new(10.0, 28.0));

        let mut filter_text = graphics::Text::new(
            graphics::TextFragment::new("green = filter-predicted trajectory")
                .color(Color::new(0.3, 1.0, 0.3, 0.7)),
        );
        filter_text.set_font("LiberationMono").set_scale(11.);
        canvas.draw(&filter_text, Vec2::new(10.0, 42.0));

        Ok(())
    }

    fn draw_impact_zone(
        &self,
        ctx: &mut Context,
        canvas: &mut Canvas,
        prediction: &ImpactPrediction,
    ) -> Result<(), GameError> {
        let mean_world = Vec2::new(prediction.mean_x, 0.0);
        let mean_screen = self.world_to_screen.transform_point2(mean_world);

        let half_width = 2.0 * prediction.std_x;
        let left_world = Vec2::new(prediction.mean_x - half_width, 0.0);
        let right_world = Vec2::new(prediction.mean_x + half_width, 0.0);
        let left_screen = self.world_to_screen.transform_point2(left_world);
        let right_screen = self.world_to_screen.transform_point2(right_world);

        let bar_color = Color::new(1.0, 0.65, 0.0, 0.6);

        let bar = graphics::Mesh::new_rectangle(
            ctx,
            graphics::DrawMode::fill(),
            Rect::new(
                left_screen.x,
                mean_screen.y - 4.0,
                right_screen.x - left_screen.x,
                8.0,
            ),
            bar_color,
        )?;
        canvas.draw(&bar, Vec2::default());

        let center_marker = graphics::Mesh::new_circle(
            ctx,
            graphics::DrawMode::fill(),
            Vec2::default(),
            4.0,
            0.2,
            Color::new(1.0, 0.65, 0.0, 1.0),
        )?;
        canvas.draw(&center_marker, Vec2::new(mean_screen.x, mean_screen.y));

        let min_world = Vec2::new(prediction.min_x, 0.0);
        let max_world = Vec2::new(prediction.max_x, 0.0);
        let min_screen = self.world_to_screen.transform_point2(min_world);
        let max_screen = self.world_to_screen.transform_point2(max_world);

        let whisker_color = Color::new(1.0, 0.65, 0.0, 0.3);
        let left_whisker = graphics::Mesh::new_line(
            ctx,
            &[
                Vec2::new(min_screen.x, mean_screen.y),
                Vec2::new(left_screen.x, mean_screen.y),
            ],
            1.5,
            whisker_color,
        )?;
        canvas.draw(&left_whisker, Vec2::default());
        let right_whisker = graphics::Mesh::new_line(
            ctx,
            &[
                Vec2::new(right_screen.x, mean_screen.y),
                Vec2::new(max_screen.x, mean_screen.y),
            ],
            1.5,
            whisker_color,
        )?;
        canvas.draw(&right_whisker, Vec2::default());

        Ok(())
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

                if let Some(tracker) = self.tracker.as_mut() {
                    let mut rng = rand::rng();
                    let noise_sigma = 5.0;
                    let noisy_pos = Vec2::new(
                        projectile.position.x + rng.random_range(-noise_sigma..=noise_sigma),
                        projectile.position.y + rng.random_range(-noise_sigma..=noise_sigma),
                    );
                    tracker.observe(noisy_pos);
                }
            } else {
                let projectile =
                    Projectile::fire_from(self.opponent_position).with_wind(self.wind.clone());

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
                    self.wind.reset();
                    if let Some(tracker) = self.tracker.as_mut() {
                        *tracker = Tracker::new(
                            (PHYSICS_FPS as f32).recip() * GAME_TIME_FACTOR,
                            1.0,
                            25.0,
                        );
                    }
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
            graphics::Text::new("Ballistic Kalman Tracker")
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
        self.render_filter_trajectory(ctx, &mut canvas)?;
        self.render_tracker_estimate(ctx, &mut canvas)?;
        self.render_predicted_impact(ctx, &mut canvas)?;
        self.render_projectile(ctx, &mut canvas)?;
        self.render_opponent(ctx, &mut canvas)?;
        self.render_trajectory_legend(ctx, &mut canvas)?;

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

fn covariance_ellipse(
    sigma_x: f32,
    sigma_y: f32,
    correlation: f32,
    num_points: usize,
) -> Vec<Vec2> {
    let mut points = Vec::with_capacity(num_points + 1);
    let cov_xy = correlation * sigma_x * sigma_y;
    let angle = 0.5 * (2.0 * cov_xy).atan2(sigma_x * sigma_x - sigma_y * sigma_y);
    let lambda1 = 0.5 * (sigma_x * sigma_x + sigma_y * sigma_y)
        + 0.5 * ((sigma_x * sigma_x - sigma_y * sigma_y).powi(2) + 4.0 * (cov_xy).powi(2)).sqrt();
    let lambda2 = 0.5 * (sigma_x * sigma_x + sigma_y * sigma_y)
        - 0.5 * ((sigma_x * sigma_x - sigma_y * sigma_y).powi(2) + 4.0 * (cov_xy).powi(2)).sqrt();
    let a = lambda1.sqrt().max(1.0);
    let b = lambda2.sqrt().max(1.0);

    for i in 0..num_points {
        let theta = (i as f32 / num_points as f32) * 2.0 * std::f32::consts::PI;
        let x = a * theta.cos();
        let y = b * theta.sin();
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        points.push(Vec2::new(x * cos_a - y * sin_a, x * sin_a + y * cos_a));
    }
    points.push(points[0]);
    points
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
            tracker: None,
            wind: Wind::new(0.0, 1.0, 0.0, 1.0),
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
            tracker: None,
            wind: Wind::new(0.0, 1.0, 0.0, 1.0),
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
