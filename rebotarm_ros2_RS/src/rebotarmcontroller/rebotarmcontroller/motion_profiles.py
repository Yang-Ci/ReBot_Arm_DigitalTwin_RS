from __future__ import annotations

import math

import numpy as np

DEFAULT_ACCELERATION_LIMIT_RAD_S2 = 4.0
DEFAULT_JERK_LIMIT_RAD_S3 = 30.0
DEFAULT_NATURAL_FREQUENCY_RAD_S = 8.0
DEFAULT_MAX_SUBSTEP_S = 0.005
DEFAULT_MAX_ELAPSED_S = 0.1


def advance_velocity_limited_reference(
    position: np.ndarray,
    velocity: np.ndarray,
    target: np.ndarray,
    velocity_limit: np.ndarray,
    elapsed: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Follow the latest leader target without a second-order settling tail.

    The hand-operated leader already supplies the motion trajectory. Bound
    each reference step by the selected speed and stop exactly at the latest
    target, including when it reverses. Acceleration is diagnostic here;
    acceleration/jerk shaping belongs to the browser trajectory profile.
    """
    q = np.asarray(position, dtype=np.float64)
    qd = np.asarray(velocity, dtype=np.float64)
    goal = np.asarray(target, dtype=np.float64)
    vlim = np.maximum(np.asarray(velocity_limit, dtype=np.float64), 0.0)
    dt = float(np.clip(elapsed, 0.0005, DEFAULT_MAX_ELAPSED_S))
    delta = goal - q
    step = np.clip(delta, -vlim * dt, vlim * dt)
    next_position = np.where(np.abs(delta) <= vlim * dt, goal, q + step)
    next_velocity = (next_position - q) / dt
    next_acceleration = (next_velocity - qd) / dt
    return next_position, next_velocity, next_acceleration


def advance_jerk_limited_reference(
    position: np.ndarray,
    velocity: np.ndarray,
    acceleration: np.ndarray,
    target: np.ndarray,
    velocity_limit: np.ndarray,
    dt: float,
    *,
    acceleration_limit: float = DEFAULT_ACCELERATION_LIMIT_RAD_S2,
    jerk_limit: float = DEFAULT_JERK_LIMIT_RAD_S3,
    natural_frequency: float = DEFAULT_NATURAL_FREQUENCY_RAD_S,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Advance a continuously re-targetable, jerk-limited joint reference.

    Browser/TCP targets may change every few milliseconds while reference
    position, velocity and acceleration stay continuous. The desired
    acceleration comes from a critically damped second-order attractor, then
    acceleration and jerk limits turn it into a bounded third-order profile.
    """
    q = np.asarray(position, dtype=np.float64)
    qd = np.asarray(velocity, dtype=np.float64)
    qdd = np.asarray(acceleration, dtype=np.float64)
    goal = np.asarray(target, dtype=np.float64)
    vlim = np.maximum(np.asarray(velocity_limit, dtype=np.float64), 0.0)
    step_dt = float(np.clip(dt, 0.0005, 0.02))
    amax = max(float(acceleration_limit), 1e-6)
    jmax = max(float(jerk_limit), 1e-6)
    omega = max(float(natural_frequency), 1e-3)

    delta = goal - q
    desired_acceleration = np.clip(
        omega * omega * delta - 2.0 * omega * qd,
        -amax,
        amax,
    )

    max_acceleration_change = jmax * step_dt
    next_acceleration = qdd + np.clip(
        desired_acceleration - qdd,
        -max_acceleration_change,
        max_acceleration_change,
    )
    next_acceleration = np.clip(next_acceleration, -amax, amax)
    next_velocity = qd + 0.5 * (qdd + next_acceleration) * step_dt
    next_velocity = np.clip(next_velocity, -vlim, vlim)
    next_position = q + 0.5 * (qd + next_velocity) * step_dt

    settled = (
        (np.abs(goal - next_position) <= 1e-5)
        & (np.abs(next_velocity) <= 1e-3)
        & (np.abs(next_acceleration) <= max_acceleration_change)
    )
    finished = settled
    next_position = np.where(finished, goal, next_position)
    next_velocity = np.where(finished, 0.0, next_velocity)
    next_acceleration = np.where(finished, 0.0, next_acceleration)
    return next_position, next_velocity, next_acceleration


def advance_jerk_limited_reference_over_elapsed(
    position: np.ndarray,
    velocity: np.ndarray,
    acceleration: np.ndarray,
    target: np.ndarray,
    velocity_limit: np.ndarray,
    elapsed: float,
    *,
    max_substep: float = DEFAULT_MAX_SUBSTEP_S,
    max_elapsed: float = DEFAULT_MAX_ELAPSED_S,
    acceleration_limit: float = DEFAULT_ACCELERATION_LIMIT_RAD_S2,
    jerk_limit: float = DEFAULT_JERK_LIMIT_RAD_S3,
    natural_frequency: float = DEFAULT_NATURAL_FREQUENCY_RAD_S,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Advance by wall-clock time without turning a late callback into a jump.

    The hardware sender can run slower than its nominal rate when CAN writes
    take longer than one period or a ROS command briefly owns the command
    lock.  Integrating only a capped single step makes the trajectory run in
    slow motion.  Split the elapsed wall time into bounded substeps instead so
    jerk and acceleration limits remain valid while trajectory timing stays
    tied to real time.
    """
    safe_max_substep = float(np.clip(max_substep, 0.0005, 0.02))
    safe_max_elapsed = max(float(max_elapsed), safe_max_substep)
    total_dt = float(np.clip(elapsed, 0.0005, safe_max_elapsed))
    step_count = max(1, math.ceil(total_dt / safe_max_substep))
    step_dt = total_dt / step_count

    next_position = np.asarray(position, dtype=np.float64)
    next_velocity = np.asarray(velocity, dtype=np.float64)
    next_acceleration = np.asarray(acceleration, dtype=np.float64)
    for _ in range(step_count):
        next_position, next_velocity, next_acceleration = (
            advance_jerk_limited_reference(
                next_position,
                next_velocity,
                next_acceleration,
                target,
                velocity_limit,
                step_dt,
                acceleration_limit=acceleration_limit,
                jerk_limit=jerk_limit,
                natural_frequency=natural_frequency,
            )
        )
    return next_position, next_velocity, next_acceleration
