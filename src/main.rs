mod projectile;
mod sim;
mod tracker;

use crate::projectile::{Projectile, Wind};
use crate::sim::{
    PHYSICS_FPS, PredictionChannel, covariance_ellipse, floor_height_from_offset, physics_dt,
    projectile_in_bounds, world_to_screen_matrix,
};
use crate::tracker::{ImpactPrediction, Tracker};
use ggez::glam::*;
use ggez::graphics::{self, Canvas, Color, Drawable, Rect};
use ggez::winit::dpi::PhysicalSize;
use ggez::{Context, GameResult};
use ggez::{GameError, conf, event};
use rand::RngExt;
use std::time::Duration;
use std::{env, path};

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
    impact_prediction: Option<ImpactPrediction>,
    filter_trajectory: Option<Vec<Vec2>>,
    frames_since_impact_update: u64,
    prediction_channel: PredictionChannel,
}

impl MainState {
    fn new(ctx: &mut Context) -> GameResult<MainState> {
        ctx.gfx.add_font(
            "LiberationMono",
            graphics::FontData::from_path(ctx, "/LiberationMono-Regular.ttf")?,
        );

        let dt = physics_dt();
        let tracker = Some(Tracker::new(dt, 1.0, 25.0));
        let wind = Wind::new(5.0, 0.5, 2.0, 3.0);
        let prediction_channel = PredictionChannel::new();

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
            impact_prediction: None,
            filter_trajectory: None,
            frames_since_impact_update: 0,
            prediction_channel,
        };
        Ok(s)
    }

    fn floor_height(&self) -> f32 {
        floor_height_from_offset(self.screen_offset)
    }

    fn create_transformation_matrix(&self) -> Mat3 {
        world_to_screen_matrix(
            self.window_size,
            self.world_size,
            self.screen_scale,
            self.screen_offset,
        )
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
            let screen_points: Vec<_> = trajectory
                .iter()
                .map(|&pos| self.world_to_screen.transform_point2(pos))
                .collect();
            if screen_points.len() > 1 {
                let line = graphics::Mesh::new_line(
                    ctx,
                    &screen_points,
                    2.0,
                    Color::new(0.25, 0.35, 0.65, 0.5),
                )?;
                canvas.draw(&line, Vec2::default());
            }
        }

        Ok(())
    }

    fn render_filter_trajectory(
        &self,
        ctx: &mut Context,
        canvas: &mut Canvas,
    ) -> Result<(), GameError> {
        if let Some(trajectory) = &self.filter_trajectory {
            let screen_points: Vec<_> = trajectory
                .iter()
                .map(|&pos| self.world_to_screen.transform_point2(pos))
                .collect();
            if screen_points.len() > 1 {
                let line = graphics::Mesh::new_line(
                    ctx,
                    &screen_points,
                    2.0,
                    Color::new(0.3, 1.0, 0.3, 0.2),
                )?;
                canvas.draw(&line, Vec2::default());
            }
        }

        Ok(())
    }

    fn projectile_in_bounds(&self, projectile: Vec2, radius: f32) -> bool {
        projectile_in_bounds(
            &self.world_to_screen,
            self.window_size,
            self.floor_height(),
            projectile,
            radius,
        )
    }

    fn update_transformations(&mut self) {
        self.world_to_screen = self.create_transformation_matrix();
    }

    fn render_tracker_estimate(
        &self,
        ctx: &mut Context,
        canvas: &mut Canvas,
    ) -> Result<(), GameError> {
        if let Some(tracker) = &self.tracker
            && tracker.is_initialized()
        {
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
        Ok(())
    }

    fn render_predicted_impact(
        &self,
        ctx: &mut Context,
        canvas: &mut Canvas,
    ) -> Result<(), GameError> {
        if let Some(prediction) = &self.impact_prediction {
            self.draw_impact_zone(ctx, canvas, prediction)?;
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

    fn update_filter_trajectory(&mut self) {
        let sim_dt = physics_dt();
        if let Some(tracker) = &self.tracker
            && let Some(t) = tracker.predicted_trajectory(sim_dt, 0.1, 500)
        {
            self.filter_trajectory = Some(t);
        }
    }

    fn request_impact_prediction(&mut self) {
        if let Some(tracker) = &self.tracker
            && tracker.is_initialized()
        {
            let sim_dt = physics_dt();
            self.prediction_channel.request(
                tracker.state_vector(),
                tracker.pv_covariance(),
                sim_dt,
                64,
                500,
            );
        }
    }

    fn collect_impact_prediction(&mut self) {
        if let Some(pred) = self.prediction_channel.collect() {
            self.impact_prediction = Some(pred);
        }
    }
}

impl event::EventHandler<GameError> for MainState {
    fn update(&mut self, ctx: &mut Context) -> GameResult {
        while ctx.time.check_update_time(PHYSICS_FPS) {
            let time_delta = Duration::from_secs_f32(physics_dt());

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
                        Duration::from_secs_f32(physics_dt()),
                        Duration::from_secs_f32(0.1),
                    )
                    .take_while(|&pos| self.projectile_in_bounds(pos, 0.0))
                    .collect();

                self.projectile = Some(projectile);
                self.projectile_trajectory = Some(trajectory);
            }

            // Reset the projectile if out of bounds.
            if let Some(projectile) = &self.projectile
                && !self.projectile_in_bounds(projectile.position, projectile.radius)
            {
                self.projectile = None;
                self.projectile_trajectory = None;
                self.wind.reset();
                if let Some(tracker) = self.tracker.as_mut() {
                    *tracker = Tracker::new(physics_dt(), 1.0, 25.0);
                }
                self.impact_prediction = None;
                self.filter_trajectory = None;
                self.prediction_channel.drain();
            }
        }

        self.collect_impact_prediction();

        const IMPACT_UPDATE_INTERVAL: u64 = 3;
        self.frames_since_impact_update += 1;
        if self.frames_since_impact_update >= IMPACT_UPDATE_INTERVAL {
            self.frames_since_impact_update = 0;
            self.request_impact_prediction();
            self.update_filter_trajectory();
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
            impact_prediction: None,
            filter_trajectory: None,
            frames_since_impact_update: 0,
            prediction_channel: PredictionChannel::new(),
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
            impact_prediction: None,
            filter_trajectory: None,
            frames_since_impact_update: 0,
            prediction_channel: PredictionChannel::new(),
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
