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
