# Ballistics Estimation

[![CI](https://github.com/sunsided/ballistics/actions/workflows/ci.yml/badge.svg)](https://github.com/sunsided/ballistics/actions/workflows/ci.yml)
[![license: EUPL-1.2](https://img.shields.io/badge/license-EUPL--1.2-blue.svg)](https://github.com/sunsided/unit-interval/blob/main/Cargo.toml)

A Kalman Filter playground with the following idea:

- A computer-generated adversary player fires a projectile.
- The projectile is observed by the current player, and the parameters of the projectile are estimated.
- From that, a simulation obtains the most likely area of impact.

To complicate matters:

- Varying wind speed can affect the projectile.

Questions:

- Can we estimate the mass of the projectile as well?
  - Both the mass as well as the initial firing force should determine the acceleration.
  - The mass will affect the curvature.
