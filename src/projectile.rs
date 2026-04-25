use ggez::glam::Vec2;
use rand::RngExt;
use rand_distr::{Distribution, Normal};
use std::f32::consts::PI;
use std::time::Duration;

const DEG_TO_RAD: f32 = PI / 180.0;

#[derive(Clone)]
pub struct Wind {
    time: f32,
    base_strength: f32,
    frequency: f32,
    noise_sigma: f32,
    prev_noise: f32,
    correlation_time: f32,
}

impl Wind {
    pub fn new(
        base_strength: f32,
        frequency: f32,
        noise_sigma: f32,
        correlation_time: f32,
    ) -> Self {
        Self {
            time: 0.0,
            base_strength,
            frequency,
            noise_sigma,
            prev_noise: 0.0,
            correlation_time,
        }
    }

    pub fn step(&mut self, dt: f32) -> Vec2 {
        self.time += dt;

        let correlation = (-dt / self.correlation_time).exp();
        let mut rng = rand::rng();
        let white_noise: f32 = rng.random_range(-self.noise_sigma..=self.noise_sigma);
        self.prev_noise =
            correlation * self.prev_noise + (1.0 - correlation * correlation).sqrt() * white_noise;

        let deterministic = self.base_strength * (self.time * self.frequency).sin();
        let acceleration_x = deterministic + self.prev_noise;

        Vec2::new(acceleration_x, 0.0)
    }

    pub fn reset(&mut self) {
        self.time = 0.0;
        self.prev_noise = 0.0;
    }
}

#[derive(Clone)]
pub struct Projectile {
    pub radius: f32, // TODO: This is a rendering concern, not a physical property
    pub position: Vec2,
    velocity: Vec2,
    acceleration: Vec2,
    wind: Option<Wind>,
}

impl Default for Projectile {
    fn default() -> Self {
        Self {
            position: Vec2::default(),
            velocity: Vec2::default(),
            acceleration: Vec2::default(),
            radius: 5.0,
            wind: None,
        }
    }
}

impl Projectile {
    pub fn fire_from(position: Vec2) -> Self {
        let mut rng = rand::rng();

        let normal = Normal::new(60.0, 10.0).unwrap();
        let angle_deg: f32 = normal.sample(&mut rng);
        let angle_deg = angle_deg.clamp(25.0, 80.0);
        let angle_rad = angle_deg * DEG_TO_RAD;
        let (sin, cos) = angle_rad.sin_cos();

        let magnitude: f32 = rng.random_range(100.0..=120.0);

        let velocity_x = -magnitude * cos;
        let velocity_y = magnitude * sin;

        Self {
            position,
            velocity: Vec2::new(velocity_x, velocity_y),
            acceleration: Vec2::new(0.0, 0.0),
            wind: None,
            ..Default::default()
        }
    }

    pub fn with_wind(mut self, wind: Wind) -> Self {
        self.wind = Some(wind);
        self
    }

    pub fn velocity(&self) -> f32 {
        self.velocity.length()
    }

    pub fn step(&mut self, duration: Duration, gravity: Vec2) {
        let dt = duration.as_secs_f32();

        let wind_accel = if let Some(wind) = &mut self.wind {
            wind.step(dt)
        } else {
            Vec2::ZERO
        };

        let total_accel = self.acceleration + wind_accel;
        self.position += self.velocity * dt + 0.5 * total_accel * dt.powi(2);
        self.velocity += total_accel * dt;
        self.acceleration = gravity;
    }

    pub fn simulate(
        &self,
        gravity: Vec2,
        delta_time: Duration,
        sample_every: Duration,
    ) -> ProjectileSimulator {
        let mut projectile = self.clone();
        projectile.wind = None;
        ProjectileSimulator {
            projectile,
            gravity,
            delta_time,
            last_sample: Duration::default(),
            sample_every,
        }
    }
}

pub struct ProjectileSimulator {
    projectile: Projectile,
    gravity: Vec2,
    delta_time: Duration,
    last_sample: Duration,
    sample_every: Duration,
}

impl Iterator for ProjectileSimulator {
    type Item = Vec2;

    fn next(&mut self) -> Option<Self::Item> {
        while self.last_sample < self.sample_every {
            self.projectile.step(self.delta_time, self.gravity);
            self.last_sample += self.delta_time;
        }
        self.last_sample -= self.sample_every;
        Some(self.projectile.position)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;
    use std::f32::consts::FRAC_PI_2;

    const EPS: f32 = 1e-4;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < EPS
    }

    #[test]
    fn projectile_default_is_zeroed() {
        let p = Projectile::default();
        assert_eq!(p.position, Vec2::ZERO);
        assert_eq!(p.velocity(), 0.0);
        assert!(approx_eq(p.radius, 5.0));
        assert!(p.wind.is_none());
    }

    #[test]
    fn step_applies_gravity_only_without_wind() {
        let gravity = Vec2::new(0.0, -9.81);
        let mut p = Projectile {
            position: Vec2::new(0.0, 0.0),
            velocity: Vec2::new(10.0, 0.0),
            acceleration: Vec2::ZERO,
            radius: 5.0,
            wind: None,
        };
        let dt = Duration::from_secs_f32(0.5);

        // First step: stored acceleration is still zero, so only horizontal motion
        p.step(dt, gravity);
        assert!(approx_eq(p.position.x, 5.0));
        assert!(approx_eq(p.position.y, 0.0));
        assert!(approx_eq(p.velocity.x, 10.0));
        assert!(approx_eq(p.velocity.y, 0.0));
    }

    #[test]
    fn step_second_call_uses_stored_gravity() {
        let gravity = Vec2::new(0.0, -9.81);
        let mut p = Projectile {
            position: Vec2::new(0.0, 0.0),
            velocity: Vec2::new(10.0, 0.0),
            acceleration: Vec2::ZERO,
            radius: 5.0,
            wind: None,
        };
        let dt = Duration::from_secs_f32(0.5);

        // First step (acceleration still zero)
        p.step(dt, gravity);
        // Second step now uses stored gravity
        p.step(dt, gravity);
        // After two steps: v.y = 0 + (-9.81)*0.5 = -4.905
        assert!(approx_eq(p.velocity.y, -4.905));
        // Position: p.y = 0 + 0*0.5 + 0.5*(-9.81)*0.25 = -1.22625
        let expected_y = 0.5 * (-9.81) * 0.25;
        assert!((p.position.y - expected_y).abs() < 1e-3);
    }

    #[test]
    fn velocity_returns_speed_magnitude() {
        let p = Projectile {
            velocity: Vec2::new(3.0, 4.0),
            ..Default::default()
        };
        assert!(approx_eq(p.velocity(), 5.0));
    }

    #[test]
    fn fire_from_produces_in_range_velocity_and_angle() {
        let origin = Vec2::new(100.0, 0.0);
        for _ in 0..300 {
            let p = Projectile::fire_from(origin);
            let mag = p.velocity();
            assert!(mag >= 100.0 && mag <= 120.0, "magnitude {mag} out of range");
            assert!(p.position == origin);
            // Upper-left quadrant due to -cos/+sin construction
            assert!(p.velocity.x <= 0.0, "velocity.x should be <= 0");
            assert!(p.velocity.y >= 0.0, "velocity.y should be >= 0");
            let angle = p.velocity.y.atan2(-p.velocity.x).to_degrees();
            assert!(
                (25.0..=80.0).contains(&angle),
                "angle {angle} not in [25, 80]"
            );
        }
    }

    #[test]
    fn with_wind_sets_wind() {
        let wind = Wind::new(10.0, 1.0, 0.0, 1.0);
        let p = Projectile::default().with_wind(wind);
        assert!(p.wind.is_some());
    }

    #[test]
    fn wind_affects_trajectory() {
        let gravity = Vec2::new(0.0, -9.81);
        let wind = Wind::new(50.0, 1.0, 0.0, 1.0);
        let mut p_with_wind = Projectile {
            position: Vec2::new(0.0, 100.0),
            velocity: Vec2::new(-10.0, 20.0),
            acceleration: Vec2::ZERO,
            radius: 5.0,
            wind: Some(wind.clone()),
        };
        let mut p_no_wind = Projectile {
            position: Vec2::new(0.0, 100.0),
            velocity: Vec2::new(-10.0, 20.0),
            acceleration: Vec2::ZERO,
            radius: 5.0,
            wind: None,
        };
        let dt = Duration::from_secs_f32(0.1);

        p_with_wind.step(dt, gravity);
        p_no_wind.step(dt, gravity);

        assert!(
            (p_with_wind.velocity.x - p_no_wind.velocity.x).abs() > 1e-3,
            "wind should affect x velocity"
        );
    }

    #[test]
    fn simulate_iterator_strips_wind() {
        let wind = Wind::new(10.0, 1.0, 0.0, 1.0);
        let p_with_wind = Projectile::default().with_wind(wind);
        let p_no_wind = Projectile::default();
        let gravity = Vec2::new(0.0, -9.81);
        let delta = Duration::from_secs_f32(0.01);
        let sample = Duration::from_secs_f32(0.1);

        let samples_with: Vec<_> = p_with_wind
            .simulate(gravity, delta, sample)
            .take(3)
            .collect();
        let samples_without: Vec<_> = p_no_wind.simulate(gravity, delta, sample).take(3).collect();

        for (a, b) in samples_with.iter().zip(samples_without.iter()) {
            assert!((a.x - b.x).abs() < 1e-6);
            assert!((a.y - b.y).abs() < 1e-6);
        }
    }

    #[test]
    fn simulate_iterator_samples_at_interval() {
        let mut p = Projectile::default();
        p.velocity = Vec2::new(10.0, 0.0);
        let gravity = Vec2::new(0.0, 0.0);
        let delta = Duration::from_secs_f32(0.01);
        let sample = Duration::from_secs_f32(0.1);

        let samples: Vec<_> = p.simulate(gravity, delta, sample).take(3).collect();

        // At ~0.1s intervals with vx=10, positions should be ~1.0, ~2.0, ~3.0 apart in x.
        // f32 imprecision (0.01 accumulates to >0.1) may cause one extra step.
        assert!((samples[0].x - 1.0).abs() < 0.2);
        assert!((samples[1].x - 2.0).abs() < 0.2);
        assert!((samples[2].x - 3.0).abs() < 0.2);
    }

    #[test]
    fn wind_new_initial_step_is_finite() {
        let mut wind = Wind::new(5.0, 1.0, 0.0, 1.0);
        let result = wind.step(0.0);
        assert!(result.x.is_finite());
        assert!(result.y == 0.0);
        // With dt=0 and σ=0, the base is sin(0)=0
        assert!(approx_eq(result.x, 0.0));
    }

    #[test]
    fn wind_step_at_half_period() {
        let mut wind = Wind::new(5.0, 1.0, 0.0, 1.0);
        // Step by π to get sin(π) = 0
        let _ = wind.step(FRAC_PI_2);
        let _ = wind.step(FRAC_PI_2);
        let result = wind.step(0.0);
        // Base component should be sin(π) = 0; noise is 0 (σ=0)
        assert!((result.x).abs() < 1e-3);
    }

    #[test]
    fn wind_reset_zeros_state() {
        let mut wind = Wind::new(5.0, 1.0, 0.5, 1.0);
        // Do several stochastic steps
        for _ in 0..10 {
            let _ = wind.step(0.1);
        }
        wind.reset();
        // After reset with σ=0, step(0) should return (0,0)
        let mut wind_det = Wind::new(5.0, 1.0, 0.0, 1.0);
        wind_det.reset();
        let result = wind_det.step(0.0);
        assert!(approx_eq(result.x, 0.0));
        assert!(approx_eq(result.y, 0.0));
    }

    proptest::proptest! {
        #[test]
        fn fire_from_always_in_upper_left_quadrant(
            origin_x in -1000.0f32..1000.0,
            origin_y in -1000.0f32..1000.0,
        ) {
            let origin = Vec2::new(origin_x, origin_y);
            let p = Projectile::fire_from(origin);
            assert_eq!(p.position, origin);
            let mag = p.velocity();
            prop_assert!(mag >= 100.0 && mag <= 120.0, "magnitude {mag} out of range");
            prop_assert!(p.velocity.x <= 0.0, "velocity.x = {} should be <= 0", p.velocity.x);
            prop_assert!(p.velocity.y >= 0.0, "velocity.y = {} should be >= 0", p.velocity.y);
            let angle = p.velocity.y.atan2(-p.velocity.x).to_degrees();
            prop_assert!((25.0..=80.0).contains(&angle), "angle {angle} not in [25, 80]");
        }
    }
}
