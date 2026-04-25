use ggez::glam::Vec2;
use minikalman::matrix::{AsMatrixMut, Matrix, MatrixMut};
use minikalman::prelude::{
    AsMatrix, DirectProcessNoiseCovarianceMatrixMut, EstimateCovarianceMatrix,
    MeasurementNoiseCovarianceMatrix, ObservationMatrixMut, StateTransitionMatrixMut,
};
use minikalman::regular::builder::{
    KalmanFilterBuilder, KalmanFilterObservationType, KalmanFilterType,
};
use rand_distr::Distribution;

pub const NUM_STATES: usize = 6;
const NUM_MEASUREMENTS: usize = 2;
const MAX_TRAJECTORY_POINTS: usize = 1024;

pub struct Tracker {
    filter: KalmanFilterType<NUM_STATES, f32>,
    measurement: KalmanFilterObservationType<NUM_STATES, NUM_MEASUREMENTS, f32>,
    #[allow(dead_code)]
    dt: f32,
    initialized: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct ImpactPrediction {
    pub mean_x: f32,
    pub std_x: f32,
    pub min_x: f32,
    pub max_x: f32,
}

impl Tracker {
    pub fn new(dt: f32, process_noise: f32, measurement_noise: f32) -> Self {
        let builder = KalmanFilterBuilder::<NUM_STATES, f32>::default();
        let mut filter = builder.build();
        let mut measurement = builder.observations().build::<NUM_MEASUREMENTS>();

        filter.state_vector_mut().as_matrix_mut().apply(|x| {
            for i in 0..NUM_STATES {
                x[i] = 0.0;
            }
        });

        filter
            .state_transition_mut()
            .as_matrix_mut()
            .apply(|a| build_state_transition(dt, a));

        filter.estimate_covariance_mut().as_matrix_mut().apply(|p| {
            p.set_all(0.0);
            p.set(0, 0, 1.0);
            p.set(1, 1, 1.0);
            p.set(2, 2, 10.0);
            p.set(3, 3, 10.0);
            p.set(4, 4, 100.0);
            p.set(5, 5, 100.0);
        });

        filter
            .direct_process_noise_mut()
            .as_matrix_mut()
            .apply(|q| {
                q.set_all(0.0);
                for i in 0..NUM_STATES {
                    q.set(i, i, process_noise);
                }
            });

        measurement
            .observation_matrix_mut()
            .as_matrix_mut()
            .apply(|h| {
                h.set_all(0.0);
                h.set(0, 0, 1.0);
                h.set(1, 1, 1.0);
            });

        measurement
            .measurement_noise_covariance_mut()
            .as_matrix_mut()
            .apply(|r| {
                r.set_all(0.0);
                r.set(0, 0, measurement_noise);
                r.set(1, 1, measurement_noise);
            });

        Self {
            filter,
            measurement,
            dt,
            initialized: false,
        }
    }

    #[allow(dead_code)]
    pub fn update_matrices(&mut self, dt: f32) {
        if dt != self.dt {
            self.dt = dt;
            self.filter
                .state_transition_mut()
                .as_matrix_mut()
                .apply(|a| build_state_transition(dt, a));
        }
    }

    pub fn initialize(&mut self, position: Vec2) {
        filter_set_state(&mut self.filter, position, Vec2::ZERO, Vec2::ZERO);
        self.initialized = true;
    }

    pub fn observe(&mut self, position: Vec2) {
        if !self.initialized {
            self.initialize(position);
            return;
        }

        self.measurement.measurement_vector_mut().apply(|z| {
            z[0] = position.x;
            z[1] = position.y;
        });

        self.filter.predict();
        self.filter.correct(&mut self.measurement);
    }

    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    pub fn estimated_position(&self) -> Vec2 {
        let state = self.filter.state_vector();
        let x = state.as_matrix().get(0, 0);
        let y = state.as_matrix().get(1, 0);
        Vec2::new(x, y)
    }

    #[allow(dead_code)]
    pub fn estimated_velocity(&self) -> Vec2 {
        let state = self.filter.state_vector();
        let vx = state.as_matrix().get(2, 0);
        let vy = state.as_matrix().get(3, 0);
        Vec2::new(vx, vy)
    }

    pub fn position_covariance_1sigma(&self) -> (f32, f32, f32) {
        let cov = self.filter.estimate_covariance();
        let p00 = cov.as_matrix().get(0, 0);
        let p11 = cov.as_matrix().get(1, 1);
        let p01 = cov.as_matrix().get(0, 1);
        let sigma_x = p00.sqrt();
        let sigma_y = p11.sqrt();
        let correlation = if sigma_x > 0.0 && sigma_y > 0.0 {
            p01 / (sigma_x * sigma_y)
        } else {
            0.0
        };
        (sigma_x, sigma_y, correlation)
    }

    pub fn predicted_trajectory(
        &self,
        sim_dt: f32,
        sample_interval: f32,
        max_steps: usize,
    ) -> Option<Vec<Vec2>> {
        if !self.initialized {
            return None;
        }

        let mut state = [0.0f32; NUM_STATES];
        self.filter.state_vector().as_matrix().inspect(|m| {
            for (i, s) in state.iter_mut().enumerate() {
                *s = m.get(i, 0);
            }
        });

        let mut trajectory = Vec::new();
        let mut x = state[0];
        let mut y = state[1];
        let mut vx = state[2];
        let mut vy = state[3];
        let ax = state[4];
        let ay = state[5];
        let mut elapsed = 0.0f32;

        if y <= 0.0 {
            return None;
        }

        trajectory.push(Vec2::new(x, y));

        for _ in 0..max_steps {
            let prev_x = x;
            let prev_y = y;
            let old_vx = vx;
            let old_vy = vy;

            x += vx * sim_dt + 0.5 * ax * sim_dt * sim_dt;
            y += vy * sim_dt + 0.5 * ay * sim_dt * sim_dt;
            vx += ax * sim_dt;
            vy += ay * sim_dt;
            elapsed += sim_dt;

            while elapsed >= sample_interval {
                elapsed -= sample_interval;
                trajectory.push(Vec2::new(x, y));
                if trajectory.len() >= MAX_TRAJECTORY_POINTS {
                    break;
                }
            }

            if trajectory.len() >= MAX_TRAJECTORY_POINTS {
                break;
            }

            if y <= 0.0 {
                let t_hit = solve_ground_time(prev_y, old_vy, ay, sim_dt);
                let impact_x = prev_x + old_vx * t_hit + 0.5 * ax * t_hit * t_hit;
                trajectory.push(Vec2::new(impact_x, 0.0));
                break;
            }
        }

        if trajectory.len() > 1 {
            Some(trajectory)
        } else {
            None
        }
    }

    pub fn state_vector(&self) -> [f32; NUM_STATES] {
        let mut state = [0.0f32; NUM_STATES];
        self.filter.state_vector().as_matrix().inspect(|m| {
            for (i, s) in state.iter_mut().enumerate() {
                *s = m.get(i, 0);
            }
        });
        state
    }

    pub fn pv_covariance(&self) -> [[f32; 4]; 4] {
        let mut pv_cov = [[0.0f32; 4]; 4];
        self.filter.estimate_covariance().as_matrix().inspect(|m| {
            for (i, row) in pv_cov.iter_mut().enumerate() {
                for (j, cell) in row.iter_mut().enumerate() {
                    *cell = m.get(i, j);
                }
            }
        });
        pv_cov
    }

    #[allow(dead_code)]
    pub fn predict_impact(
        &self,
        sim_dt: f32,
        num_samples: usize,
        num_sim_steps: usize,
    ) -> Option<ImpactPrediction> {
        if !self.initialized {
            return None;
        }
        let state = self.state_vector();
        let pv_cov = self.pv_covariance();
        predict_impact_from_state(&state, &pv_cov, sim_dt, num_samples, num_sim_steps)
    }
}

pub fn predict_impact_from_state(
    state: &[f32; NUM_STATES],
    pv_cov: &[[f32; 4]; 4],
    sim_dt: f32,
    num_samples: usize,
    num_sim_steps: usize,
) -> Option<ImpactPrediction> {
    let pv_cholesky = cholesky_decompose_4x4(pv_cov);

    let mut rng = rand::rng();
    let normal = rand_distr::Normal::new(0.0, 1.0).unwrap();
    let mut impacts = Vec::with_capacity(num_samples);

    for _ in 0..num_samples {
        let sample = sample_pv_trajectory(state, &pv_cholesky, &mut rng, &normal);

        if let Some(impact_x) = simulate_trajectory(&sample, sim_dt, num_sim_steps) {
            impacts.push(impact_x);
        }
    }

    if impacts.is_empty() {
        return None;
    }

    let sum: f32 = impacts.iter().sum();
    let mean = sum / impacts.len() as f32;
    let variance: f32 =
        impacts.iter().map(|x| (x - mean) * (x - mean)).sum::<f32>() / impacts.len() as f32;
    let std = variance.sqrt();
    let min_x = impacts.iter().cloned().fold(f32::INFINITY, f32::min);
    let max_x = impacts.iter().cloned().fold(f32::NEG_INFINITY, f32::max);

    Some(ImpactPrediction {
        mean_x: mean,
        std_x: std,
        min_x,
        max_x,
    })
}

fn filter_set_state(
    filter: &mut KalmanFilterType<NUM_STATES, f32>,
    position: Vec2,
    velocity: Vec2,
    acceleration: Vec2,
) {
    filter.state_vector_mut().as_matrix_mut().apply(|x| {
        x[0] = position.x;
        x[1] = position.y;
        x[2] = velocity.x;
        x[3] = velocity.y;
        x[4] = acceleration.x;
        x[5] = acceleration.y;
    });
}

fn build_state_transition(dt: f32, a: &mut impl MatrixMut<6, 6, f32>) {
    a.set_all(0.0);
    let dt2_half = 0.5 * dt * dt;

    a.set(0, 0, 1.0);
    a.set(0, 2, dt);
    a.set(0, 4, dt2_half);

    a.set(1, 1, 1.0);
    a.set(1, 3, dt);
    a.set(1, 5, dt2_half);

    a.set(2, 2, 1.0);
    a.set(2, 4, dt);

    a.set(3, 3, 1.0);
    a.set(3, 5, dt);

    a.set(4, 4, 1.0);
    a.set(5, 5, 1.0);
}

fn cholesky_decompose_4x4(matrix: &[[f32; 4]; 4]) -> [[f32; 4]; 4] {
    let mut l = [[0.0f32; 4]; 4];
    for i in 0..4 {
        for j in 0..=i {
            let sum: f32 = (0..j).map(|k| l[i][k] * l[j][k]).sum();
            if i == j {
                let val = matrix[i][i] - sum;
                l[i][j] = if val > 0.0 { val.sqrt() } else { 0.0 };
            } else {
                l[i][j] = if l[j][j] > 1e-10 {
                    (matrix[i][j] - sum) / l[j][j]
                } else {
                    0.0
                };
            }
        }
    }
    l
}

fn sample_pv_trajectory(
    state: &[f32; NUM_STATES],
    cholesky: &[[f32; 4]; 4],
    rng: &mut impl rand::Rng,
    normal: &rand_distr::Normal<f32>,
) -> [f32; NUM_STATES] {
    let mut sample = *state;
    let mut z = [0.0f32; 4];
    for z_i in z.iter_mut() {
        *z_i = normal.sample(rng);
    }

    for i in 0..4 {
        let noise: f32 = (0..=i).map(|j| cholesky[i][j] * z[j]).sum();
        sample[i] += noise;
    }
    sample
}

fn simulate_trajectory(state: &[f32; NUM_STATES], sim_dt: f32, max_steps: usize) -> Option<f32> {
    let mut x = state[0];
    let mut y = state[1];
    let mut vx = state[2];
    let mut vy = state[3];
    let ax = state[4];
    let ay = state[5];

    if y <= 0.0 {
        return Some(x);
    }

    for _ in 0..max_steps {
        let prev_x = x;
        let prev_y = y;
        let prev_vx = vx;
        let prev_vy = vy;

        x += vx * sim_dt + 0.5 * ax * sim_dt * sim_dt;
        y += vy * sim_dt + 0.5 * ay * sim_dt * sim_dt;
        vx += ax * sim_dt;
        vy += ay * sim_dt;

        if y <= 0.0 {
            let t_hit = solve_ground_time(prev_y, prev_vy, ay, sim_dt);
            let impact_x = prev_x + prev_vx * t_hit + 0.5 * ax * t_hit * t_hit;
            return Some(impact_x);
        }
    }

    None
}

fn solve_ground_time(y0: f32, vy0: f32, ay: f32, dt: f32) -> f32 {
    let a = 0.5 * ay;
    let b = vy0;
    let c = y0;

    if a.abs() < 1e-10 {
        if b.abs() < 1e-10 {
            return 0.0;
        }
        return (-c / b).clamp(0.0, dt);
    }

    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return dt;
    }

    let sqrt_disc = disc.sqrt();
    let t1 = (-b - sqrt_disc) / (2.0 * a);
    let t2 = (-b + sqrt_disc) / (2.0 * a);

    if t1 > 0.0 && t1 <= dt {
        t1
    } else if t2 > 0.0 && t2 <= dt {
        t2
    } else {
        dt
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use minikalman::prelude::StateTransitionMatrix;
    use proptest::prelude::*;

    const EPS: f32 = 1e-4;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < EPS
    }

    #[test]
    fn predicted_trajectory_capped_at_max_points() {
        let mut tracker = Tracker::new(0.167, 1.0, 25.0);
        tracker.initialize(Vec2::new(0.0, 100.0));
        tracker.observe(Vec2::new(10.0, 150.0));
        tracker.observe(Vec2::new(30.0, 200.0));

        let result = tracker.predicted_trajectory(0.167, 0.01, 10000);
        assert!(result.is_some());
        assert!(result.unwrap().len() <= MAX_TRAJECTORY_POINTS);
    }

    #[test]
    fn predicted_trajectory_returns_none_when_uninitialized() {
        let tracker = Tracker::new(0.167, 1.0, 25.0);
        assert!(tracker.predicted_trajectory(0.167, 0.1, 500).is_none());
    }

    #[test]
    fn predicted_trajectory_returns_none_below_ground() {
        let mut tracker = Tracker::new(0.167, 1.0, 25.0);
        tracker.initialize(Vec2::new(0.0, -5.0));
        assert!(tracker.predicted_trajectory(0.167, 0.1, 500).is_none());
    }

    // --- solve_ground_time tests ---

    #[test]
    fn solve_ground_time_linear_case() {
        // ay ≈ 0, vy0 < 0, y0 > 0 → expect -y0/vy0 clamped to dt
        let t = solve_ground_time(100.0, -10.0, 0.0, 20.0);
        assert!(approx_eq(t, 10.0));
    }

    #[test]
    fn solve_ground_time_quadratic_picks_positive_root_in_range() {
        // y0=10, vy0=0, ay=-9.81 → t = sqrt(2*10/9.81) ≈ 1.43
        let dt = 5.0;
        let t = solve_ground_time(10.0, 0.0, -9.81, dt);
        assert!(t > 0.0 && t <= dt);
        let y_at_t = 10.0 + 0.0 * t + 0.5 * (-9.81) * t * t;
        assert!(y_at_t.abs() < 0.01);
    }

    #[test]
    fn solve_ground_time_degenerate_zero_velocity_zero_accel() {
        let t = solve_ground_time(10.0, 0.0, 0.0, 5.0);
        assert!(approx_eq(t, 0.0));
    }

    #[test]
    fn solve_ground_time_no_real_root_returns_dt() {
        // discriminant < 0: y0>0, vy0=0, ay>0 (accelerating upward)
        let t = solve_ground_time(10.0, 0.0, 1.0, 5.0);
        assert!(approx_eq(t, 5.0));
    }

    // --- Cholesky tests ---

    #[test]
    fn cholesky_decomposes_identity_to_identity() {
        let identity = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ];
        let l = cholesky_decompose_4x4(&identity);
        for i in 0..4 {
            for j in 0..4 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((l[i][j] - expected).abs() < 1e-5);
            }
        }
    }

    #[test]
    fn cholesky_decomposes_known_psd_matrix() {
        // Build A = L * L^T from a known L
        let l_orig = [
            [2.0, 0.0, 0.0, 0.0],
            [1.0, 3.0, 0.0, 0.0],
            [0.0, 1.0, 2.0, 0.0],
            [1.0, 0.0, 1.0, 4.0],
        ];
        let mut a = [[0.0f32; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                a[i][j] = (0..4).map(|k| l_orig[i][k] * l_orig[j][k]).sum();
            }
        }
        let l = cholesky_decompose_4x4(&a);
        // Reconstruct and compare
        for i in 0..4 {
            for j in 0..4 {
                let recon: f32 = (0..4).map(|k| l[i][k] * l[j][k]).sum();
                assert!(
                    (recon - a[i][j]).abs() < 1e-3,
                    "reconstruction mismatch at [{i}][{j}]: {recon} vs {a_ij}",
                    a_ij = a[i][j]
                );
            }
        }
    }

    #[test]
    fn cholesky_handles_non_psd_gracefully() {
        // Matrix with negative diagonal entry
        let bad = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ];
        let l = cholesky_decompose_4x4(&bad);
        // l[1][1] should be 0 since -1 is negative
        assert!(approx_eq(l[1][1], 0.0));
        // No panic should occur
        assert!(l[0][0] > 0.0);
    }

    // --- build_state_transition test via Tracker ---

    #[test]
    fn build_state_transition_matches_kinematics() {
        let dt = 0.5;
        let tracker = Tracker::new(dt, 1.0, 1.0);
        let a = tracker.filter.state_transition().as_matrix();
        let mut matrix = [[0.0f32; 6]; 6];
        a.inspect(|m| {
            for i in 0..6 {
                for j in 0..6 {
                    matrix[i][j] = m.get(i, j);
                }
            }
        });
        let dt2 = 0.5 * dt * dt;
        assert!(approx_eq(matrix[0][0], 1.0));
        assert!(approx_eq(matrix[0][2], dt));
        assert!(approx_eq(matrix[0][4], dt2));
        assert!(approx_eq(matrix[1][1], 1.0));
        assert!(approx_eq(matrix[1][3], dt));
        assert!(approx_eq(matrix[1][5], dt2));
        assert!(approx_eq(matrix[2][2], 1.0));
        assert!(approx_eq(matrix[2][4], dt));
        assert!(approx_eq(matrix[3][3], 1.0));
        assert!(approx_eq(matrix[3][5], dt));
        assert!(approx_eq(matrix[4][4], 1.0));
        assert!(approx_eq(matrix[5][5], 1.0));
    }

    // --- simulate_trajectory tests ---

    #[test]
    fn simulate_trajectory_returns_start_x_if_already_underground() {
        let mut state = [0.0f32; NUM_STATES];
        state[0] = 42.0;
        state[1] = -5.0;
        let result = simulate_trajectory(&state, 0.01, 1000);
        assert_eq!(result, Some(42.0));
    }

    #[test]
    fn simulate_trajectory_hits_ground_symmetric_parabola() {
        let mut state = [0.0f32; NUM_STATES];
        state[0] = 0.0;
        state[1] = 0.001;
        state[2] = 10.0;
        state[3] = 10.0;
        state[5] = -9.81;
        let result = simulate_trajectory(&state, 0.001, 10000);
        assert!(result.is_some());
        let impact = result.unwrap();
        // Expected ≈ 2*vx*vy/|ay| ≈ 2*10*10/9.81 ≈ 20.387
        assert!((impact - 20.387).abs() < 0.5);
    }

    #[test]
    fn simulate_trajectory_times_out() {
        let mut state = [0.0f32; NUM_STATES];
        state[0] = 0.0;
        state[1] = 100.0;
        state[2] = 10.0;
        // ay=0, vy=0 → never falls
        let result = simulate_trajectory(&state, 0.01, 100);
        assert!(result.is_none());
    }

    // --- sample_pv_trajectory test ---

    #[test]
    fn sample_pv_trajectory_is_identity_with_zero_cholesky() {
        let state = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let cholesky = [[0.0f32; 4]; 4];
        let mut rng = rand::rng();
        let normal = rand_distr::Normal::new(0.0, 1.0).unwrap();
        let sample = sample_pv_trajectory(&state, &cholesky, &mut rng, &normal);
        for i in 0..6 {
            assert!(approx_eq(sample[i], state[i]));
        }
    }

    // --- Tracker surface tests ---

    #[test]
    fn tracker_is_not_initialized_on_construction() {
        let tracker = Tracker::new(0.1, 1.0, 1.0);
        assert!(!tracker.is_initialized());
    }

    #[test]
    fn initialize_sets_flag() {
        let mut tracker = Tracker::new(0.1, 1.0, 1.0);
        tracker.initialize(Vec2::new(0.0, 0.0));
        assert!(tracker.is_initialized());
    }

    #[test]
    fn observe_before_initialize_initializes_from_first_observation() {
        let mut tracker = Tracker::new(0.1, 1.0, 1.0);
        tracker.observe(Vec2::new(10.0, 20.0));
        assert!(tracker.is_initialized());
        let pos = tracker.estimated_position();
        assert!(approx_eq(pos.x, 10.0));
        assert!(approx_eq(pos.y, 20.0));
        let vel = tracker.estimated_velocity();
        assert!(approx_eq(vel.x, 0.0));
        assert!(approx_eq(vel.y, 0.0));
    }

    #[test]
    fn observe_updates_position_estimate() {
        let mut tracker = Tracker::new(0.01, 1.0, 25.0);
        tracker.initialize(Vec2::new(0.0, 100.0));
        for _ in 0..20 {
            tracker.observe(Vec2::new(5.0, 105.0));
        }
        let pos = tracker.estimated_position();
        // Should have moved toward the observation
        assert!(pos.y > 100.0 && pos.y < 105.0);
    }

    #[test]
    fn position_covariance_1sigma_returns_initial_values() {
        let mut tracker = Tracker::new(0.1, 1.0, 1.0);
        tracker.initialize(Vec2::new(0.0, 0.0));
        let (sigma_x, sigma_y, correlation) = tracker.position_covariance_1sigma();
        assert!(approx_eq(sigma_x, 1.0));
        assert!(approx_eq(sigma_y, 1.0));
        assert!(approx_eq(correlation, 0.0));
    }

    #[test]
    fn state_vector_and_pv_covariance_round_trip() {
        let mut tracker = Tracker::new(0.01, 1.0, 25.0);
        tracker.initialize(Vec2::new(5.0, 10.0));
        for _ in 0..10 {
            tracker.observe(Vec2::new(5.5, 10.5));
        }
        let state = tracker.state_vector();
        let pos = tracker.estimated_position();
        assert!((state[0] - pos.x).abs() < 1e-6);
        assert!((state[1] - pos.y).abs() < 1e-6);

        let pv_cov = tracker.pv_covariance();
        let (sigma_x, sigma_y, _) = tracker.position_covariance_1sigma();
        assert!((pv_cov[0][0].sqrt() - sigma_x).abs() < 1e-6);
        assert!((pv_cov[1][1].sqrt() - sigma_y).abs() < 1e-6);
    }

    #[test]
    fn estimated_velocity_returns_nonzero_when_moving() {
        let mut tracker = Tracker::new(0.01, 1.0, 25.0);
        tracker.initialize(Vec2::new(0.0, 0.0));
        // Drive in a straight line
        for i in 0..20 {
            tracker.observe(Vec2::new(i as f32 * 0.1, 0.0));
        }
        let vel = tracker.estimated_velocity();
        assert!(vel.length() > 0.0);
    }

    #[test]
    fn predict_impact_returns_none_when_uninitialized() {
        let tracker = Tracker::new(0.1, 1.0, 1.0);
        assert!(tracker.predict_impact(0.01, 64, 500).is_none());
    }

    #[test]
    fn predict_impact_returns_a_prediction_for_airborne_projectile() {
        let mut tracker = Tracker::new(0.01, 10.0, 25.0);
        tracker.initialize(Vec2::new(0.0, 100.0));
        // Drive observations downward to induce negative velocity estimate
        for i in 0..20 {
            let y = 100.0 - (i as f32) * 2.0;
            tracker.observe(Vec2::new(0.0, y));
        }
        let pred = tracker.predict_impact(0.01, 64, 500);
        assert!(pred.is_some());
        let pred = pred.unwrap();
        assert!(pred.mean_x.is_finite());
        assert!(pred.min_x <= pred.mean_x);
        assert!(pred.mean_x <= pred.max_x);
        assert!(pred.std_x >= 0.0);
    }

    #[test]
    fn update_matrices_updates_dt() {
        let mut tracker = Tracker::new(0.1, 1.0, 1.0);
        let old_a02 = {
            let a = tracker.filter.state_transition().as_matrix();
            let mut val = 0.0;
            a.inspect(|m| val = m.get(0, 2));
            val
        };
        assert!(approx_eq(old_a02, 0.1));

        tracker.update_matrices(0.5);
        let new_a02 = {
            let a = tracker.filter.state_transition().as_matrix();
            let mut val = 0.0;
            a.inspect(|m| val = m.get(0, 2));
            val
        };
        assert!(approx_eq(new_a02, 0.5));
    }

    // --- predict_impact_from_state tests ---

    #[test]
    fn predict_impact_from_state_is_none_when_no_samples_hit_ground() {
        let mut state = [0.0f32; NUM_STATES];
        state[0] = 0.0;
        state[1] = 1000.0; // high y
        state[2] = 0.0;
        state[3] = 0.0; // no vertical velocity
        state[4] = 0.0;
        state[5] = 0.0; // no gravity
        let pv_cov = [[0.0f32; 4]; 4];
        let result = predict_impact_from_state(&state, &pv_cov, 0.01, 64, 100);
        assert!(result.is_none());
    }

    #[test]
    fn predict_impact_from_state_mean_close_to_deterministic_impact() {
        let mut state = [0.0f32; NUM_STATES];
        state[0] = 0.0;
        state[1] = 10.0;
        state[2] = 10.0;
        state[3] = 10.0;
        state[5] = -9.81;
        let pv_cov = [[0.0f32; 4]; 4]; // zero covariance
        let result = predict_impact_from_state(&state, &pv_cov, 0.001, 64, 10000);
        assert!(result.is_some());
        let pred = result.unwrap();
        assert!(pred.std_x < 1e-5);
        assert!((pred.mean_x - pred.min_x).abs() < 1e-5);
        assert!((pred.mean_x - pred.max_x).abs() < 1e-5);
        // Compare with deterministic simulate_trajectory
        let deterministic = simulate_trajectory(&state, 0.001, 10000).unwrap();
        assert!((pred.mean_x - deterministic).abs() < 1e-3);
    }

    // --- proptest property tests ---

    proptest::proptest! {
        #![proptest_config(ProptestConfig { cases: 128, .. Default::default() })]

        #[test]
        fn cholesky_roundtrip_psd(
            entries in prop::collection::vec(-5.0f32..5.0f32, 10),
        ) {
            // Build a random lower-triangular L with positive diagonals
            let mut l = [[0.0f32; 4]; 4];
            let mut idx = 0;
            for i in 0..4 {
                for j in 0..=i {
                    let val = entries[idx];
                    idx += 1;
                    l[i][j] = if i == j { val.abs() + 0.1 } else { val };
                }
            }
            // Form A = L * L^T
            let mut a = [[0.0f32; 4]; 4];
            for i in 0..4 {
                for j in 0..4 {
                    a[i][j] = (0..4).map(|k| l[i][k] * l[j][k]).sum();
                }
            }
            let l_prime = cholesky_decompose_4x4(&a);
            // Reconstruct and compare
            for i in 0..4 {
                for j in 0..4 {
                    let recon: f32 = (0..4).map(|k| l_prime[i][k] * l_prime[j][k]).sum();
                    prop_assert!((recon - a[i][j]).abs() < 1e-3,
                        "cholesky roundtrip mismatch at [{i}][{j}]: {recon} vs {}", a[i][j]);
                }
            }
        }

        #[test]
        fn solve_ground_time_in_range(
            y0 in 0.1f32..100.0,
            vy0 in -50.0f32..50.0,
            ay in -20.0f32..-0.1,
            dt in 0.001f32..1.0,
        ) {
            let t = solve_ground_time(y0, vy0, ay, dt);
            prop_assert!(t >= 0.0 && t <= dt, "t={t} not in [0, {dt}]");
            let y_at_t = y0 + vy0 * t + 0.5 * ay * t * t;
            prop_assert!(y_at_t.abs() < 1.0 || (t - dt).abs() < 1e-5,
                "y_at_t={y_at_t} is not ≈ 0 and t≠dt");
        }

        #[test]
        fn simulate_trajectory_impact_x_is_finite(
            y0 in 0.1f32..100.0,
            vx in -50.0f32..50.0,
            vy0 in -50.0f32..50.0,
            ay in -20.0f32..-0.1,
        ) {
            let mut state = [0.0f32; NUM_STATES];
            state[0] = 0.0;
            state[1] = y0;
            state[2] = vx;
            state[3] = vy0;
            state[5] = ay;
            if let Some(impact_x) = simulate_trajectory(&state, 0.01, 10000) {
                prop_assert!(impact_x.is_finite(), "impact_x={impact_x} is not finite");
            }
        }
    }
}
