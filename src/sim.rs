use crate::tracker::{ImpactPrediction, NUM_STATES, predict_impact_from_state};
use ggez::glam::{Mat3, Vec2};
use ggez::winit::dpi::PhysicalSize;
use std::sync::mpsc;

pub const PHYSICS_FPS: u32 = 60;
pub const GAME_TIME_FACTOR: f32 = 10.0;

pub fn physics_dt() -> f32 {
    (PHYSICS_FPS as f32).recip() * GAME_TIME_FACTOR
}

pub fn covariance_ellipse(
    sigma_x: f32,
    sigma_y: f32,
    correlation: f32,
    num_points: usize,
) -> Vec<Vec2> {
    debug_assert!(num_points > 0, "num_points must be > 0");
    if num_points == 0 {
        return Vec::new();
    }
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

pub fn world_to_screen_matrix(
    window_size: PhysicalSize<u32>,
    world_size: Vec2,
    screen_scale: Vec2,
    screen_offset: Vec2,
) -> Mat3 {
    let screen_width = window_size.width as f32;
    let screen_height = window_size.height as f32;

    let scale_x = screen_width / world_size.x * screen_scale.x;
    let scale_y = screen_height / world_size.y * screen_scale.y;

    let scale_matrix = Mat3::from_scale(Vec2::new(scale_x, -scale_y));
    let translation_matrix =
        Mat3::from_translation(screen_offset + Vec2::new(0.0, screen_height * screen_scale.y));

    translation_matrix * scale_matrix
}

pub fn floor_height_from_offset(screen_offset: Vec2) -> f32 {
    -screen_offset.y
}

pub fn projectile_in_bounds(
    world_to_screen: &Mat3,
    window_size: PhysicalSize<u32>,
    floor_height: f32,
    projectile: Vec2,
    radius: f32,
) -> bool {
    let pos = world_to_screen.transform_point2(projectile);

    !(pos.x <= radius
        || pos.y <= radius
        || pos.y > (window_size.height as f32 - floor_height + radius))
}

pub struct PredictionRequest {
    pub state: [f32; NUM_STATES],
    pub pv_cov: [[f32; 4]; 4],
    pub sim_dt: f32,
    pub num_samples: usize,
    pub num_sim_steps: usize,
}

pub struct PredictionChannel {
    tx: mpsc::SyncSender<PredictionRequest>,
    rx: mpsc::Receiver<Option<ImpactPrediction>>,
    pending: bool,
}

impl PredictionChannel {
    pub fn new() -> Self {
        let (tx, rx): (mpsc::SyncSender<PredictionRequest>, _) = mpsc::sync_channel(1);
        let (result_tx, result_rx) = mpsc::channel();
        std::thread::spawn(move || {
            while let Ok(req) = rx.recv() {
                let result = predict_impact_from_state(
                    &req.state,
                    &req.pv_cov,
                    req.sim_dt,
                    req.num_samples,
                    req.num_sim_steps,
                );
                let _ = result_tx.send(result);
            }
        });
        Self {
            tx,
            rx: result_rx,
            pending: false,
        }
    }

    pub fn request(
        &mut self,
        state: [f32; NUM_STATES],
        pv_cov: [[f32; 4]; 4],
        sim_dt: f32,
        num_samples: usize,
        num_sim_steps: usize,
    ) {
        if self.pending {
            return;
        }
        let request = PredictionRequest {
            state,
            pv_cov,
            sim_dt,
            num_samples,
            num_sim_steps,
        };
        if self.tx.try_send(request).is_ok() {
            self.pending = true;
        }
    }

    pub fn collect(&mut self) -> Option<ImpactPrediction> {
        if let Ok(result) = self.rx.try_recv() {
            self.pending = false;
            return result;
        }
        None
    }

    pub fn is_pending(&self) -> bool {
        self.pending
    }
}

impl Default for PredictionChannel {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f32 = 1e-4;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < EPS
    }

    #[test]
    fn physics_dt_is_consistent_with_constants() {
        let dt = physics_dt();
        let expected = (PHYSICS_FPS as f32).recip() * GAME_TIME_FACTOR;
        assert!(approx_eq(dt, expected));
    }

    #[test]
    fn floor_height_from_offset_negates_y() {
        let offset = Vec2::new(10.0, -50.0);
        assert!(approx_eq(floor_height_from_offset(offset), 50.0));
    }

    #[test]
    fn world_to_screen_matrix_identity_case() {
        let window_size = PhysicalSize::new(640, 480);
        let world_size = Vec2::new(640.0, 480.0);
        let screen_scale = Vec2::ONE;
        let screen_offset = Vec2::ZERO;

        let matrix = world_to_screen_matrix(window_size, world_size, screen_scale, screen_offset);
        let bottom_left = matrix.transform_point2(Vec2::ZERO);
        assert_eq!(bottom_left, Vec2::new(0.0, 480.0));

        let top_of_bottom_left = matrix.transform_point2(Vec2::new(0.0, 100.0));
        assert_eq!(top_of_bottom_left, Vec2::new(0.0, 380.0));

        let top_right = matrix.transform_point2(Vec2::new(100.0, 100.0));
        assert_eq!(top_right, Vec2::new(100.0, 380.0));
    }

    #[test]
    fn world_to_screen_matrix_offset_and_scaled() {
        let window_size = PhysicalSize::new(640, 480);
        let world_size = Vec2::new(640.0 * 2.0, 480.0 * 2.0);
        let screen_scale = Vec2::ONE;
        let screen_offset = Vec2::new(1000.0, 2000.0);

        let matrix = world_to_screen_matrix(window_size, world_size, screen_scale, screen_offset);
        let bottom_left = matrix.transform_point2(Vec2::ZERO);
        assert_eq!(bottom_left, Vec2::new(1000.0, 2000.0 + 480.0));

        let top_of_bottom_left = matrix.transform_point2(Vec2::new(0.0, 100.0));
        assert_eq!(top_of_bottom_left, Vec2::new(1000.0, 2000.0 + 480.0 - 50.0));

        let top_right = matrix.transform_point2(Vec2::new(100.0, 100.0));
        assert_eq!(top_right, Vec2::new(1000.0 + 50.0, 2000.0 + 480.0 - 50.0));
    }

    #[test]
    fn projectile_in_bounds_true_for_center() {
        let window_size = PhysicalSize::new(640, 480);
        let world_size = Vec2::new(640.0, 480.0);
        let screen_offset = Vec2::new(0.0, -100.0);
        let matrix = world_to_screen_matrix(window_size, world_size, Vec2::ONE, screen_offset);
        let floor_height = floor_height_from_offset(screen_offset);

        assert!(projectile_in_bounds(
            &matrix,
            window_size,
            floor_height,
            Vec2::new(320.0, 240.0),
            5.0
        ));
    }

    #[test]
    fn projectile_in_bounds_false_above_ceiling() {
        let window_size = PhysicalSize::new(640, 480);
        let world_size = Vec2::new(640.0, 480.0);
        let screen_offset = Vec2::new(0.0, -100.0);
        let matrix = world_to_screen_matrix(window_size, world_size, Vec2::ONE, screen_offset);
        let floor_height = floor_height_from_offset(screen_offset);

        // y=600 in world → screen_y = -(600) + 480 - 100 = -220 < radius
        // Actually let's use a point that's clearly above the ceiling
        assert!(!projectile_in_bounds(
            &matrix,
            window_size,
            floor_height,
            Vec2::new(320.0, 500.0),
            5.0
        ));
    }

    #[test]
    fn projectile_in_bounds_false_below_floor() {
        let window_size = PhysicalSize::new(640, 480);
        let world_size = Vec2::new(640.0, 480.0);
        let screen_offset = Vec2::new(0.0, -100.0);
        let matrix = world_to_screen_matrix(window_size, world_size, Vec2::ONE, screen_offset);
        let floor_height = floor_height_from_offset(screen_offset);

        // y=-10 in world → screen_y = -(-10) + 480 - 100 = 490 > 480 - 100 + radius
        assert!(!projectile_in_bounds(
            &matrix,
            window_size,
            floor_height,
            Vec2::new(320.0, -10.0),
            5.0
        ));
    }

    #[test]
    fn projectile_in_bounds_false_left_of_screen() {
        let window_size = PhysicalSize::new(640, 480);
        let world_size = Vec2::new(640.0, 480.0);
        let screen_offset = Vec2::new(0.0, -100.0);
        let matrix = world_to_screen_matrix(window_size, world_size, Vec2::ONE, screen_offset);
        let floor_height = floor_height_from_offset(screen_offset);

        assert!(!projectile_in_bounds(
            &matrix,
            window_size,
            floor_height,
            Vec2::new(-10.0, 240.0),
            5.0
        ));
    }

    #[test]
    fn covariance_ellipse_produces_closed_loop() {
        let points = covariance_ellipse(10.0, 5.0, 0.0, 32);
        assert_eq!(points.len(), 33);
        assert_eq!(points.first(), points.last());
    }

    #[test]
    fn covariance_ellipse_axis_aligned_for_zero_correlation() {
        let points = covariance_ellipse(10.0, 5.0, 0.0, 32);
        // First point (theta=0) should be on +x axis
        let first = &points[0];
        assert!(first.x > 0.0);
        assert!(first.y.abs() < 1.0);
        // Point at num_points/4 (theta=pi/2) should be on +y axis
        let quarter = &points[8];
        assert!(quarter.y > 0.0);
        assert!(quarter.x.abs() < 1.0);
    }

    #[test]
    fn covariance_ellipse_degenerate_small_sigmas_uses_min_radius_1() {
        let points = covariance_ellipse(0.001, 0.001, 0.0, 32);
        for p in &points {
            assert!(p.length() >= 0.99);
        }
    }

    #[test]
    fn prediction_channel_request_sets_pending_and_collect_clears_it() {
        let mut channel = PredictionChannel::new();
        assert!(!channel.is_pending());

        let mut state = [0.0f32; NUM_STATES];
        state[0] = 0.0;
        state[1] = 100.0;
        state[2] = 10.0;
        state[3] = 10.0;
        state[5] = -9.81;

        channel.request(state, [[0.0; 4]; 4], 0.01, 64, 10000);
        assert!(channel.is_pending());

        // Wait for the worker to process and collect without sleep-based polling
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
        while channel.is_pending() {
            let result = channel.collect();
            if result.is_some() {
                break;
            }
            assert!(std::time::Instant::now() < deadline, "prediction timed out");
            std::thread::yield_now();
        }
        assert!(!channel.is_pending());
    }

    #[test]
    fn prediction_channel_request_is_noop_while_pending() {
        let mut channel = PredictionChannel::new();

        let mut state = [0.0f32; NUM_STATES];
        state[0] = 0.0;
        state[1] = 100.0;
        state[2] = 10.0;
        state[3] = 10.0;
        state[5] = -9.81;

        channel.request(state, [[0.0; 4]; 4], 0.01, 64, 10000);
        assert!(channel.is_pending());

        // Second request while pending should be a no-op
        channel.request(state, [[0.0; 4]; 4], 0.01, 64, 10000);
        // Still pending (the first request is still pending)
        assert!(channel.is_pending());
    }

    #[test]
    fn prediction_channel_collect_yields_none_when_channel_empty() {
        let mut channel = PredictionChannel::new();
        let result = channel.collect();
        assert!(result.is_none());
    }
}
