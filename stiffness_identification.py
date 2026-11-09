"""Human stiffness from robot perturbations, without a force/torque sensor.

The FR3 only estimates the external wrench (O_F_ext_hat_K, from its joint torque sensors and dynamics model), so two
independent estimates are provided:

  wrench difference : the robot moves the handle by a small step and holds; K = -dF/dx from the change of the
                      estimated wrench between a window before the step and a window in the hold. The same step
                      recorded with nobody holding the handle (empty run) is subtracted to remove model errors.
  robot as spring   : the robot runs Cartesian impedance with a known stiffness K_r and its equilibrium is shifted by
                      dx_d; with the hand on the handle it only moves dx, so at rest K_h dx = K_r (dx_d - dx).
                      Uses measured positions and the commanded K_r only, no wrench.

Sign: O_F_ext_hat_K is the wrench the environment applies to the robot. A hand resisting a displacement dx pushes back
with -K dx, so K = -dF/dx.
Translational and rotational steps share the code: positions in m and forces in N, or rotation vectors in rad and
torques in Nm. Rows of dx / df are steps, columns are axes of the robot base frame.
"""

import numpy as np
from scipy.spatial.transform import Rotation

G = 9.81


def window_mean(t, y, t0, t1):
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    sel = (t >= t0) & (t < t1)
    if not sel.any():
        raise ValueError(f"no samples in window [{t0:.3f}, {t1:.3f}) s")
    return y[sel].mean(axis=0)


def step_change(t, y, onset_s, pre_s=(-0.3, -0.05), hold_s=(0.4, 0.8)):
    """Mean of y in the hold window minus its mean in the pre window; windows are relative to the step onset."""
    return (window_mean(t, y, onset_s + hold_s[0], onset_s + hold_s[1])
            - window_mean(t, y, onset_s + pre_s[0], onset_s + pre_s[1]))


def rotation_change(t, quat_xyzw, onset_s, pre_s=(-0.3, -0.05), hold_s=(0.4, 0.8)):
    """Rotation vector (rad, base frame) from the mean orientation before the step to the mean orientation in the hold."""
    t = np.asarray(t, dtype=float)
    rot = Rotation.from_quat(np.asarray(quat_xyzw, dtype=float))
    pre = (t >= onset_s + pre_s[0]) & (t < onset_s + pre_s[1])
    hold = (t >= onset_s + hold_s[0]) & (t < onset_s + hold_s[1])
    if not pre.any() or not hold.any():
        raise ValueError("no samples in the pre or hold window")
    return (rot[hold].mean() * rot[pre].mean().inv()).as_rotvec()


def steps_from_recording(t, position, wrench, onsets_s, pre_s=(-0.3, -0.05), hold_s=(0.4, 0.8)):
    """dx and df (one row per step) from a recording of end-effector position and estimated external force.
    For rotational steps pass the orientation as quaternions (x, y, z, w) and the torque part of the wrench."""
    position = np.asarray(position, dtype=float)
    rotational = position.ndim == 2 and position.shape[1] == 4
    dx, df = [], []
    for onset in onsets_s:
        dx.append(rotation_change(t, position, onset, pre_s, hold_s) if rotational
                  else step_change(t, position, onset, pre_s, hold_s))
        df.append(step_change(t, wrench, onset, pre_s, hold_s))
    return np.array(dx), np.array(df)


def usable_steps(dx, df, min_dx, min_df):
    """Mask of steps large enough to fit: |dx| >= min_dx and |df| >= min_df (Euclidean norms per step)."""
    dx = np.atleast_2d(dx)
    df = np.atleast_2d(df)
    return (np.linalg.norm(dx, axis=1) >= min_dx) & (np.linalg.norm(df, axis=1) >= min_df)


def _fit(x, y):
    """Least-squares M with y_i = M x_i for rows x_i, y_i; requires the steps to span every axis."""
    x = np.atleast_2d(np.asarray(x, dtype=float))
    y = np.atleast_2d(np.asarray(y, dtype=float))
    if x.shape != y.shape:
        raise ValueError(f"shape mismatch {x.shape} vs {y.shape}")
    if np.linalg.matrix_rank(x) < x.shape[1]:
        raise ValueError("steps do not span all axes; perturb along every axis (both directions)")
    m = (y.T @ x) @ np.linalg.inv(x.T @ x)
    ss_res = np.sum((y - x @ m.T) ** 2)
    ss_tot = np.sum(y ** 2)
    return m, (1.0 - ss_res / ss_tot) if ss_tot > 0 else np.nan


def stiffness_from_wrench(dx, df, df_empty=None, symmetric=True):
    """Stiffness matrix from wrench-difference steps. df_empty: the same steps without a hand, subtracted row by row.
    Returns K (symmetrised by default) and the through-origin R^2 of the unsymmetrised fit."""
    df = np.atleast_2d(np.asarray(df, dtype=float))
    if df_empty is not None:
        df = df - np.atleast_2d(np.asarray(df_empty, dtype=float))
    k, r2 = _fit(dx, -df)
    return (0.5 * (k + k.T) if symmetric else k), r2


def stiffness_from_spring(dx_d, dx, k_robot, symmetric=True):
    """Stiffness matrix of the hand from equilibrium shifts dx_d and the resulting rest displacements dx, with the
    robot under impedance control of stiffness k_robot (scalar or matrix): K_h dx = K_r (dx_d - dx)."""
    dx_d = np.atleast_2d(np.asarray(dx_d, dtype=float))
    dx = np.atleast_2d(np.asarray(dx, dtype=float))
    k_r = np.asarray(k_robot, dtype=float)
    if k_r.ndim == 0:
        k_r = k_r * np.eye(dx.shape[1])
    k, r2 = _fit(dx, (dx_d - dx) @ k_r.T)
    return (0.5 * (k + k.T) if symmetric else k), r2


def axis_stiffness(k_matrix, axis):
    """Stiffness along a unit direction: u^T K u."""
    u = np.asarray(axis, dtype=float)
    u = u / np.linalg.norm(u)
    return float(u @ np.asarray(k_matrix, dtype=float) @ u)


def is_positive_definite(k_matrix):
    k = np.asarray(k_matrix, dtype=float)
    return bool(np.all(np.linalg.eigvalsh(0.5 * (k + k.T)) > 0))


def sag_stiffness(mass_kg, dz_m):
    """Check of the robot's own stiffness: a known mass hung on the end effector sags dz under impedance control."""
    return mass_kg * G / abs(dz_m)
