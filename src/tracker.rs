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
