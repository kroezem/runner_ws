# Vendored RF2O Laser Odometry

Vendored to keep this workspace self-contained and preserve the local runtime integration fix.

- Upstream: https://github.com/MAPIRlab/rf2o_laser_odometry.git
- Branch: `ros2`
- Base commit: `60a2f0f`
- Local modification: the `/tf` → `/tf_disabled` remap that prevents RF2O from publishing `odom` → `base_link` (which the EKF owns), working around RF2O ignoring `publish_tf: false` at runtime.
- Local modification: RF2O defers first-scan initialization and retries on later scans until the laser-to-base transform is available, rather than using a default transform after a failed lookup.
