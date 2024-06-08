# Ballistics Estimation

A Kalman Filter playground with the following idea:

- A computer-generated adversary player fires a projectile.
- The projectile is observed by the current player, and the parameters of the projectile are estimated.
- From that, a simulation obtains the most likely area of impact.

To complicate matters:

- Varying wind speed can affect the projectile.

Questions:

- Can we estimate the mass of the projectile as well?
  - Both the mass as well, as the initial firing force should determine the acceleration.
  - The mass will affect the curvature.
