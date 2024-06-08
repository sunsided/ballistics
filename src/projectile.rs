use ggez::glam::Vec2;
use rand::Rng;
use std::f32::consts::PI;
use std::time::Duration;

#[derive(Clone)]
pub struct Projectile {
    pub radius: f32, // TODO: This is a rendering concern, not a physical property
    pub position: Vec2,
    velocity: Vec2,
    acceleration: Vec2,
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
        let mut rng = rand::thread_rng();
        let angle_deg: f32 = rng.gen_range(45.0..=80.0);
        let angle_rad = angle_deg * PI / 180.0;
        let magnitude: f32 = rng.gen_range(50.0..=150.0);

        let (sin, cos) = angle_rad.sin_cos();
        let velocity_x = -magnitude * cos;
        let velocity_y = magnitude * sin;

        Self {
            position,
            velocity: Vec2::new(velocity_x, velocity_y),
            acceleration: Vec2::new(0.0, 0.0),
            ..Default::default()
        }
    }

    pub fn velocity(&self) -> f32 {
        self.velocity.length()
    }

    pub fn step(&mut self, duration: Duration, gravity: Vec2) {
        let duration = duration.as_secs_f32();
        self.position += self.velocity * duration + 0.5 * self.acceleration * duration.powi(2);
        self.velocity += self.acceleration * duration;
        self.acceleration = gravity;
    }

    pub fn simulate(
        &self,
        gravity: Vec2,
        delta_time: Duration,
        sample_every: Duration,
    ) -> ProjectileSimulator {
        ProjectileSimulator {
            projectile: self.clone(),
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
