# Vendored RF2O Laser Odometry

Vendored to keep this workspace self-contained and preserve the local runtime integration fix.

- Upstream: https://github.com/MAPIRlab/rf2o_laser_odometry.git
- Branch: `ros2`
- Base commit: `60a2f0f`
- Local modification: the `/tf` → `/tf_disabled` remap that prevents RF2O from publishing `odom` → `base_link` (which the EKF owns), working around RF2O ignoring `publish_tf: false` at runtime.
- Local modification: RF2O defers first-scan initialization and retries on later scans until the laser-to-base transform is available, rather than using a default transform after a failed lookup.
- Local modification: `/rf2o/diag` appends low-overhead update timing and publication-latency instrumentation. Its layout is:
  - `0`: solve valid flag
  - `1`: effective scan `dt`
  - `2`: solver valid-point count
  - `3`: unweighted residual SSE
  - `4..12`: increment covariance, 3x3 row-major in `vx, vy, wz` order
  - `13..21`: weighted information matrix (`AtA`), 3x3 row-major
  - `22`: total update wall time in milliseconds (`std::chrono::steady_clock`)
  - `23`: total update thread CPU time in milliseconds (`CLOCK_THREAD_CPUTIME_ID`)
  - `24`: scan-header-stamp to imminent odometry publication age in seconds
  - `25`: incoming scan range count
  - `26`: incoming finite-range count
  - `27`: `createImagePyramid()` wall time in milliseconds
  - `28`: cumulative `performWarping()` wall time in milliseconds
  - `29`: cumulative `solveSystemNonLinear()` wall time in milliseconds
  - `30`: `PoseUpdate()` wall time in milliseconds
  - `31`: incoming scan header stamp in seconds
  - `32`: previous processed scan header stamp in seconds
  - `33`: effective scan interval in seconds (index 31 minus index 32)

  Indices `0..21` retain their prior meaning and ordering. A warning, throttled to once per five seconds, reports the incoming/previous scan timing and range counts when update wall time exceeds 200 ms or publication age exceeds 300 ms. Comparing wall time with per-thread CPU time distinguishes active computation from blocking, descheduling, or other off-CPU delay.

## Stall investigation notes

- The executable uses a single-threaded `spin_some()` / `process()` / `rclcpp::Rate::sleep()` loop. Scan callbacks cannot run during an RF2O update, and the most recent depth-one scan replaces intermediate scans before the next `spin_some()`.
- There are no RF2O update-path mutexes, condition-variable waits, sleeps, retries, or unbounded loops. Startup transform lookup is non-blocking and is retried on subsequent scans; its warning is throttled.
- The update performs dynamic Eigen matrix resize/allocation, five IRLS rounds with `LDLT`, covariance `inverse()`, eigendecomposition, QR solves, and another inverse. These operations are bounded by the scan width (roughly 503 points) but are candidates for unexpectedly expensive computation; the wall/CPU and stage fields identify that case.
- The former per-update INFO logs (execution time plus laser and robot pose) could block in logging or terminal/journald backpressure and produced scan-rate output. The timing log was removed and the pose logs were reduced to DEBUG.
- Width is fixed from the initialization scan. A later wider scan (for example 505 after 503) is truncated to the initialized width. A later shorter scan can be indexed past its range vector in the callback and in `odometryCalculation()`. This behavior is unchanged by the diagnostic patch.
