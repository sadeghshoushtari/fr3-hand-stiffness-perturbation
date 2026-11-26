"""Line-of-action model of the right arm: posture -> arm Jacobian J and muscle Jacobian J_m.

Follows Sierotowicz et al. (ICORR 2022) and Sierotowicz & Castellini (BPEE 2023):
- every muscle is a string routed through points fixed to the torso, the humerus or the forearm;
- J_m = dl/dq (muscle length over joint motion), so muscle forces f give joint torques tau = -J_m^T f;
- the shoulder is a point with 3 revolute axes (the world axes through the glenohumeral centre), because the
  anatomical axes are hard to measure; the elbow is one flexion axis; the wrist is braced, the endpoint is the
  handle centre in the hand.
- J = dx/dq for the handle position, so endpoint forces give joint torques J^T F.

Joint coordinates q = (dx, dy, dz, d_elbow) are local: a small rotation of the upper arm about the world axes,
applied to the measured posture, and the change of elbow flexion. Every derivative is taken at q = 0, so the model
needs no Euler angles and works in any posture the IMUs report.

Frames (all right-handed). Upper-arm frame: origin at the glenohumeral centre, x anterior, y along the humerus
towards the shoulder, z lateral (= elbow flexion axis). Forearm frame: origin at the elbow centre, equal to the
upper-arm frame rotated about z by the elbow flexion angle (0 = straight arm). Torso frame: origin at the
glenohumeral centre, same axes as the upper-arm frame when the arm hangs straight down.

Lines that go around a joint (triceps over the elbow) wrap over a sphere of radius R at the joint centre, as in the
sphere routing of the MyoCeption model (ICORR 2022), here as the exact shortest path (straight line when it clears
the sphere, great-circle arc when it does not), so the moment arm stays continuous.

For the 6x6 hand stiffness (hand_model.py), hand_jacobian adds forearm twist, wrist flexion and wrist deviation to q
(HAND_JOINTS) and the hand's rotation to the endpoint.

Muscle points are given for a reference arm (upper arm 0.30 m, forearm 0.27 m) and scaled to the subject. They are
fitted to the MoBL-ARMS model (see MUSCLES), so the moment arms follow that model in 3D, not only at the elbow.
"""

from dataclasses import dataclass, field

import numpy as np

REF_UPPER_ARM_M = 0.30
REF_FOREARM_M = 0.27

# name: (points [(frame, (x, y, z) m) or ("wrap_elbow", R m)], optimal fibre length l0 m, maximum isometric force N)
# Muscles, l0 and F_max: MoBL-ARMS (Holzbaur et al. 2005, OpenSim 4.1 release), the shoulder and elbow muscles that
# Hu et al. 2011 used for endpoint stiffness, plus ECRL and pronator teres (elbow flexors). The points were fitted to
# the MoBL-ARMS lengths and moment arms over shoulder elevation 0-100 deg, plane of elevation -30-130 deg, humeral
# rotation -45-75 deg, elbow 20-140 deg, forearm neutral (moment-arm rms error 0.1-0.6 cm, teres minor 1.3 cm;
# fitted with OpenSim, script kept outside the repository). Each point stays within 5 cm of its MoBL-ARMS position.
# Points on the clavicle and scapula are fixed to the torso; the shoulder girdle motion is absorbed by the fit.
# l0 and F_max set each muscle's short-range stiffness k = gamma F / l0 and its force for the geometric term.
# A ("wrap_elbow", R) entry wraps the segment between its neighbours over the elbow.
MUSCLES = {
    "deltoid_anterior": ([("humerus", (0.008, -0.122, 0.011)), ("humerus", (0.016, -0.086, 0.005)),
                       ("torso", (0.045, 0.023, -0.018)), ("torso", (0.010, 0.023, -0.069))], 0.098, 1218.9),
    "deltoid_middle": ([("humerus", (0.003, -0.140, 0.012)), ("humerus", (0.005, -0.113, 0.025)),
                     ("torso", (0.001, 0.025, 0.021)), ("torso", (-0.008, 0.033, -0.006))], 0.108, 1103.5),
    "deltoid_posterior": ([("torso", (-0.055, 0.023, -0.035)), ("torso", (-0.048, -0.008, 0.027)),
                        ("humerus", (0.002, -0.055, 0.010))], 0.137, 201.6),
    "supraspinatus": ([("humerus", (0.005, 0.012, 0.026)), ("humerus", (0.009, 0.013, 0.020)),
                    ("torso", (-0.039, 0.045, -0.046)), ("torso", (-0.042, 0.005, -0.069))], 0.068, 499.2),
    "infraspinatus": ([("humerus", (-0.004, 0.007, 0.022)), ("torso", (-0.057, -0.023, -0.015)),
                    ("torso", (-0.064, -0.040, -0.052))], 0.075, 1075.8),
    "subscapularis": ([("humerus", (0.013, 0.008, -0.015)), ("humerus", (0.016, 0.008, -0.016)),
                    ("torso", (-0.001, -0.075, -0.084)), ("torso", (-0.066, -0.026, -0.071))], 0.087, 1306.9),
    "teres_minor": ([("humerus", (0.000, -0.012, 0.023)), ("humerus", (0.004, -0.006, 0.023)),
                  ("torso", (-0.071, -0.092, 0.022)), ("torso", (-0.081, -0.071, -0.053))], 0.074, 269.5),
    "teres_major": ([("humerus", (0.009, -0.056, -0.004)), ("humerus", (0.004, -0.038, -0.009)),
                  ("torso", (-0.115, -0.147, -0.048)), ("torso", (-0.085, -0.103, -0.068))], 0.162, 144.0),
    "pectoralis_clavicular": ([("humerus", (0.012, -0.043, 0.009)), ("humerus", (0.017, -0.034, -0.001)),
                            ("torso", (0.035, -0.005, -0.034)), ("torso", (0.031, 0.010, -0.096))], 0.144, 444.3),
    "pectoralis_sternal": ([("humerus", (0.013, -0.044, 0.009)), ("humerus", (0.014, -0.047, -0.006)),
                         ("torso", (0.050, -0.049, -0.042)), ("torso", (0.057, -0.047, -0.152))], 0.139, 658.3),
    "pectoralis_ribs": ([("humerus", (0.013, -0.045, 0.009)), ("humerus", (0.012, -0.055, -0.009)),
                      ("torso", (0.050, -0.073, -0.040)), ("torso", (0.087, -0.120, -0.136))], 0.139, 498.1),
    "latissimus_thoracic": ([("humerus", (0.010, -0.036, -0.006)), ("humerus", (0.005, -0.035, -0.011)),
                          ("torso", (-0.071, -0.094, -0.019)), ("torso", (-0.071, -0.121, -0.166))], 0.254, 290.5),
    "latissimus_lumbar": ([("humerus", (0.009, -0.042, -0.005)), ("humerus", (0.003, -0.048, -0.011)),
                        ("torso", (-0.067, -0.132, -0.010)), ("torso", (-0.046, -0.194, -0.167))], 0.232, 317.5),
    "latissimus_iliac": ([("humerus", (0.011, -0.041, -0.004)), ("humerus", (0.004, -0.055, -0.011)),
                       ("torso", (-0.043, -0.160, -0.010)), ("torso", (-0.045, -0.257, -0.166))], 0.279, 189.0),
    "coracobrachialis": ([("torso", (0.022, -0.007, -0.037)), ("torso", (0.022, -0.030, -0.024)),
                       ("humerus", (0.008, -0.194, -0.013)), ("humerus", (0.004, -0.156, -0.002))], 0.093, 208.2),
    "triceps_long": ([("torso", (-0.024, -0.013, -0.007)), ("humerus", (-0.056, -0.169, 0.014)),
                   ("wrap_elbow", 0.020), ("forearm", (-0.020, 0.012, -0.002))], 0.134, 771.8),
    "triceps_lateral": ([("humerus", (-0.000, -0.182, 0.022)), ("wrap_elbow", 0.020),
                      ("forearm", (-0.015, 0.020, -0.005))], 0.114, 717.5),
    "triceps_medial": ([("humerus", (-0.000, -0.193, -0.020)), ("wrap_elbow", 0.020),
                     ("forearm", (-0.015, 0.020, 0.005))], 0.114, 717.5),
    "anconeus": ([("humerus", (-0.020, -0.295, 0.000)), ("wrap_elbow", 0.020),
               ("forearm", (-0.012, -0.001, -0.000))], 0.027, 283.2),
    "biceps_long": ([("torso", (-0.025, 0.004, -0.022)), ("torso", (-0.028, 0.023, -0.025)),
                  ("humerus", (0.018, 0.014, 0.002)), ("humerus", (0.027, -0.135, 0.008)),
                  ("forearm", (-0.006, -0.046, -0.010)), ("forearm", (-0.005, -0.050, 0.009))], 0.116, 525.1),
    "biceps_short": ([("torso", (0.022, -0.005, -0.037)), ("torso", (0.018, -0.028, -0.024)),
                   ("humerus", (0.014, -0.094, -0.012)), ("humerus", (0.034, -0.130, 0.030)),
                   ("forearm", (-0.009, -0.049, -0.031)), ("forearm", (-0.005, -0.050, 0.009))], 0.132, 316.8),
    "brachialis": ([("humerus", (0.000, -0.231, 0.000)), ("forearm", (0.000, -0.024, 0.000))], 0.086, 1177.4),
    "brachioradialis": ([("humerus", (-0.015, -0.207, 0.012)), ("forearm", (0.023, -0.144, 0.017)),
                      ("forearm", (0.023, -0.236, 0.001))], 0.173, 276.0),
    "extensor_carpi_radialis_longus": ([("humerus", (-0.006, -0.268, 0.022)), ("forearm", (0.000, -0.103, 0.021)),
                                     ("forearm", (0.015, -0.293, 0.002))], 0.081, 337.3),
    "pronator_teres": ([("humerus", (0.001, -0.284, -0.024)), ("forearm", (0.009, -0.017, -0.021)),
                     ("forearm", (0.016, -0.125, 0.018))], 0.049, 557.2),
}

JOINTS = ("shoulder_x", "shoulder_y", "shoulder_z", "elbow")
# Hand model (hand_jacobian): the 4 arm joints plus forearm twist about the forearm axis (forearm y), then wrist flexion
# about the twisted forearm x axis and radial/ulnar deviation about its z axis, for a neutral forearm (handshake grip,
# palm facing medially). No muscle of MUSCLES crosses them: their stiffness is joint-level (hand_model.py).
HAND_JOINTS = JOINTS + ("forearm_twist", "wrist_flexion", "wrist_deviation")


def skew(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def rotvec_to_matrix(v):
    """Rodrigues: rotation vector (rad) -> rotation matrix."""
    v = np.asarray(v, dtype=float)
    angle = np.linalg.norm(v)
    if angle < 1e-12:
        return np.eye(3) + skew(v)
    k = skew(v / angle)
    return np.eye(3) + np.sin(angle) * k + (1.0 - np.cos(angle)) * k @ k


def rot_x(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


@dataclass
class Posture:
    """Measured arm posture in the world (robot base) frame."""
    upper_arm: np.ndarray  # rotation matrix of the upper-arm frame
    elbow_angle: float  # rad, 0 = straight
    shoulder: np.ndarray = field(default_factory=lambda: np.zeros(3))  # glenohumeral centre, m
    torso: np.ndarray = field(default_factory=lambda: np.eye(3))  # rotation matrix of the torso frame


def posture_from_directions(humerus_down, forearm, torso=np.eye(3), shoulder=np.zeros(3)):
    """Posture from the humerus direction (shoulder -> elbow) and forearm direction (elbow -> hand), world frame.

    Needs a bent elbow: with a straight arm the flexion axis (and the humerus roll) is undefined; use the
    upper-arm IMU orientation directly in that case.
    """
    d = np.asarray(humerus_down, dtype=float)
    f = np.asarray(forearm, dtype=float)
    d, f = d / np.linalg.norm(d), f / np.linalg.norm(f)
    axis = np.cross(d, f)
    if np.linalg.norm(axis) < 0.05:
        raise ValueError("elbow nearly straight: flexion axis undefined, use the upper-arm IMU orientation")
    z = axis / np.linalg.norm(axis)
    y = -d
    x = np.cross(y, z)
    angle = np.arctan2(np.linalg.norm(np.cross(d, f)), d @ f)
    return Posture(np.column_stack([x, y, z]), float(angle), np.asarray(shoulder, float), np.asarray(torso, float))


def wrapped_length(a, b, centre, radius):
    """Shortest path from a to b that does not enter the sphere (centre, radius). If a or b lies inside, the sphere
    shrinks to just inside the nearer point (an attachment cannot be inside the bone it wraps over)."""
    u, v = a - centre, b - centre
    du, dv = np.linalg.norm(u), np.linalg.norm(v)
    radius = min(radius, 0.99 * du, 0.99 * dv)
    angle = np.arccos(np.clip(u @ v / (du * dv), -1.0, 1.0))
    arc = angle - np.arccos(radius / du) - np.arccos(radius / dv)
    if arc <= 0:  # the straight line clears the sphere
        return float(np.linalg.norm(b - a))
    return float(np.sqrt(du ** 2 - radius ** 2) + np.sqrt(dv ** 2 - radius ** 2) + radius * arc)


class ArmModel:
    def __init__(self, upper_arm_m=REF_UPPER_ARM_M, forearm_m=REF_FOREARM_M, hand_m=0.07, muscles=None):
        """upper_arm_m: shoulder -> elbow centre; forearm_m: elbow -> wrist; hand_m: wrist -> handle centre."""
        self.upper_arm_m, self.forearm_m, self.hand_m = upper_arm_m, forearm_m, hand_m
        spec = MUSCLES if muscles is None else muscles
        self.names = tuple(spec)
        scale = {"torso": upper_arm_m / REF_UPPER_ARM_M, "humerus": upper_arm_m / REF_UPPER_ARM_M,
                 "forearm": forearm_m / REF_FOREARM_M}
        self.paths = [[(frame, p * scale["forearm"] if frame == "wrap_elbow" else scale[frame] * np.asarray(p, float))
                       for frame, p in spec[n][0]] for n in self.names]
        self.l0 = np.array([spec[n][1] for n in self.names])
        self.f_max = np.array([spec[n][2] for n in self.names])

    # --- forward kinematics at local joint coordinates q around the measured posture ---

    def _frames(self, posture, q):
        r_ua = rotvec_to_matrix(q[:3]) @ posture.upper_arm
        r_fa = r_ua @ rot_z(posture.elbow_angle + q[3])
        elbow = posture.shoulder + r_ua @ np.array([0.0, -self.upper_arm_m, 0.0])
        return r_ua, r_fa, elbow

    def handle(self, posture, q=np.zeros(4)):
        _, r_fa, elbow = self._frames(posture, q)
        return elbow + r_fa @ np.array([0.0, -(self.forearm_m + self.hand_m), 0.0])

    def hand_frame(self, posture, q=np.zeros(7)):
        """Handle centre and hand rotation at the 7 joint coordinates of HAND_JOINTS (the wrist sits forearm_m from
        the elbow, the handle centre hand_m further along the hand). At q = 0 the handle is handle(posture)."""
        _, r_fa, elbow = self._frames(posture, q[:4])
        r_hand = r_fa @ rot_y(q[4]) @ rot_x(q[5]) @ rot_z(q[6])
        wrist = elbow + r_fa @ np.array([0.0, -self.forearm_m, 0.0])
        return wrist + r_hand @ np.array([0.0, -self.hand_m, 0.0]), r_hand

    def hand_jacobian(self, posture, h=1e-6):
        """J6 (6 x 7): handle velocity (rows 0-2) and hand angular velocity (rows 3-5, world frame) per joint velocity
        of HAND_JOINTS, at q = 0. Rows 0-2 of the first 4 columns are arm_jacobian."""
        cols = []
        for k in range(7):
            dq = np.zeros(7)
            dq[k] = h
            (p1, r1), (p0, r0) = self.hand_frame(posture, dq), self.hand_frame(posture, -dq)
            w = r1 @ r0.T  # rotation from -h to +h, small: its skew part is the rotation vector
            omega = 0.5 * np.array([w[2, 1] - w[1, 2], w[0, 2] - w[2, 0], w[1, 0] - w[0, 1]])
            cols.append(np.concatenate([(p1 - p0) / (2 * h), omega / (2 * h)]))
        return np.column_stack(cols)

    def muscle_points(self, posture, q=np.zeros(4)):
        """Attachment and via points per muscle (wrap entries left out), world frame."""
        r_ua, r_fa, elbow = self._frames(posture, q)
        place = {"torso": lambda p: posture.shoulder + posture.torso @ p,
                 "humerus": lambda p: posture.shoulder + r_ua @ p,
                 "forearm": lambda p: elbow + r_fa @ p}
        return [np.array([place[frame](p) for frame, p in path if frame in place]) for path in self.paths]

    def muscle_lengths(self, posture, q=np.zeros(4)):
        _, _, elbow = self._frames(posture, q)
        lengths = []
        for path, pts in zip(self.paths, self.muscle_points(posture, q)):
            seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
            k = 0
            for frame, radius in path:
                if frame == "wrap_elbow":  # wraps the segment between the previous and the next point
                    seg[k - 1] = wrapped_length(pts[k - 1], pts[k], elbow, radius)
                else:
                    k += 1
            lengths.append(seg.sum())
        return np.array(lengths)

    # --- derivatives at q = 0 (central differences; the functions are smooth) ---

    @staticmethod
    def _jacobian(fun, q0, h):
        cols = []
        for k in range(len(q0)):
            dq = np.zeros(len(q0))
            dq[k] = h
            cols.append((fun(q0 + dq) - fun(q0 - dq)) / (2 * h))
        return np.column_stack(cols)

    def arm_jacobian(self, posture, q=np.zeros(4), h=1e-6):
        """J (3 x 4): handle velocity per joint velocity."""
        return self._jacobian(lambda x: self.handle(posture, x), np.asarray(q, float), h)

    def muscle_jacobian(self, posture, q=np.zeros(4), h=1e-6):
        """J_m (M x 4) = dl/dq. Moment arm about a joint = -J_m (positive = shortens when the joint turns +)."""
        return self._jacobian(lambda x: self.muscle_lengths(posture, x), np.asarray(q, float), h)

    def muscle_hessians(self, posture, h=1e-4):
        """d2 l_i / dq2 (M x 4 x 4): change of the moment arms with posture (muscle-force geometric stiffness)."""
        cols = []
        for k in range(4):
            dq = np.zeros(4)
            dq[k] = h
            cols.append((self.muscle_jacobian(posture, dq) - self.muscle_jacobian(posture, -dq)) / (2 * h))
        hess = np.stack(cols, axis=2)
        return 0.5 * (hess + hess.transpose(0, 2, 1))

    def load_stiffness(self, posture, force_on_hand, h=1e-4):
        """G = d(J^T F)/dq (4 x 4) for a constant force F that the environment applies to the hand (world frame)."""
        f = np.asarray(force_on_hand, float)
        g = self._jacobian(lambda x: self.arm_jacobian(posture, x).T @ f, np.zeros(4), h)
        return 0.5 * (g + g.T)

    def moment_arms(self, posture):
        """Moment arms (M x 4), m; positive = the muscle turns the joint in its + direction when it pulls."""
        return -self.muscle_jacobian(posture)
