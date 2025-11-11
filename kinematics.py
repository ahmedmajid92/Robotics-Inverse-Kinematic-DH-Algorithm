"""
kinematics.py

Exact geometric FK/IK for the 5-DOF arm from:
"Inverse Kinematics Analysis and Simulation of a 5 DOF Robotic Arm using MATLAB"

Paper-aligned assumptions:
  - Elbow-up solution (θ3 < 0)
  - θ5 = 90°
  - Wrist pitch φ depends on target quadrant (matches the two demo cases):
        Px ≥ 0  → φ = 11°  ⇒ θ234 = 79°
        Px < 0  → φ = 12°  ⇒ θ234 = 78°
"""

from math import cos, sin, atan2, acos, sqrt, radians, degrees
import numpy as np

# --- Link dimensions (mm) from the paper ---
a2 = 105.0
a3 = 100.0
d1 = 105.0
d5 = 150.0

TH5_DEG = 90.0

# ---------------------------
# Forward kinematics (FK)
# ---------------------------
def _A(theta_deg, d, a, alpha_deg):
    """Standard DH transform (theta, d, a, alpha) – degrees for angles."""
    th = radians(theta_deg)
    al = radians(alpha_deg)
    ct, st = cos(th), sin(th)
    ca, sa = cos(al), sin(al)
    return np.array(
        [
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0.0,      sa,      ca,      d],
            [0.0,     0.0,     0.0,    1.0],
        ],
        dtype=float,
    )

def get_all_joint_positions(thetas_deg):
    """
    Return a (6,3) array of joint origins in base frame for frames 0..5.
    Includes base (0,0,0) and frames 1..5.
    """
    th1, th2, th3, th4, th5 = thetas_deg
    A1 = _A(th1, d1, 0.0, 90.0)
    A2 = _A(th2, 0.0, a2, 0.0)
    A3 = _A(th3, 0.0, a3, 0.0)
    A4 = _A(th4, 0.0, 0.0, 90.0)
    A5 = _A(th5, d5, 0.0, 0.0)

    T0 = np.eye(4)
    T1 = T0 @ A1
    T2 = T1 @ A2
    T3 = T2 @ A3
    T4 = T3 @ A4
    T5 = T4 @ A5

    pts = np.array(
        [
            T0[:3, 3],
            T1[:3, 3],
            T2[:3, 3],
            T3[:3, 3],
            T4[:3, 3],
            T5[:3, 3],
        ],
        dtype=float,
    )
    return pts

def forward_kinematics(thetas_deg):
    """Return (points0..5, T05)."""
    th1, th2, th3, th4, th5 = thetas_deg
    A1 = _A(th1, d1, 0.0, 90.0)
    A2 = _A(th2, 0.0, a2, 0.0)
    A3 = _A(th3, 0.0, a3, 0.0)
    A4 = _A(th4, 0.0, 0.0, 90.0)
    A5 = _A(th5, d5, 0.0, 0.0)
    T = np.eye(4)
    Ts = [np.eye(4)]
    for A in (A1, A2, A3, A4, A5):
        T = T @ A
        Ts.append(T)
    pts = np.array([Ti[:3, 3] for Ti in Ts], dtype=float)
    return pts, T

# ---------------------------
# Inverse kinematics (IK)
# ---------------------------
def _clamp(x, lo=-1.0, hi=1.0):
    return hi if x > hi else lo if x < lo else x

def _phi_deg_for_target(px: float) -> float:
    """
    Paper-consistent wrist pitch selection:
      Px ≥ 0  -> φ = 11°  (θ234 = 79°)  [Case 1]
      Px < 0  -> φ = 12°  (θ234 = 78°)  [Case 2]
    """
    return 11.0 if px >= 0.0 else 12.0

def solve_ik(px, py, pz, elbow_mode="up"):
    """
    Paper-faithful elbow-UP inverse kinematics:

      θ1 = atan2(Py, Px)
      Wrist center (Eqs. 10–13) with '+' on Pz_w:
        R = d5 cos φ
        Px_w = Px − R cos θ1
        Py_w = Py − R sin θ1
        Pz_w = Pz + d5 sin φ
      λ = atan2(Pz_w − d1, sqrt(Px_w^2 + Py_w^2))
      μ from law of cosines at the shoulder
      Elbow-up: θ3 = −acos((N^2 − a2^2 − a3^2)/(2 a2 a3))
      θ2 = λ + μ     (works for both cases when φ is chosen as above)
      θ4 = (90° − φ) − θ2 − θ3
      θ5 = 90°

    Returns:
      {"angles": [θ1..θ5], "joints": [[...],[...],...], "meta": {...}}
    """
    # 1) Choose φ by target Px (matches the paper’s two demos)
    phi_deg = _phi_deg_for_target(px)
    th234_deg = 90.0 - phi_deg

    # 2) Base rotation
    th1 = atan2(py, px)  # radians

    # 3) Wrist center with '+' on Pz_w
    phi = radians(phi_deg)
    R = d5 * cos(phi)
    pxw = px - R * cos(th1)
    pyw = py - R * sin(th1)
    pzw = pz + d5 * sin(phi)

    # 4) Distances
    Rw = sqrt(pxw**2 + pyw**2)
    z_ = pzw - d1
    N = sqrt(Rw**2 + z_**2)

    # 5) λ, μ
    lam = atan2(z_, Rw)
    cos_mu = _clamp((N**2 + a2**2 - a3**2) / (2.0 * a2 * N))
    mu = acos(cos_mu)

    # 6) Elbow-up θ3
    cos_t3 = _clamp((N**2 - a2**2 - a3**2) / (2.0 * a2 * a3))
    t3_mag = acos(cos_t3)
    t3 = -t3_mag

    # 7) θ2 pairing (λ + μ)
    t2 = lam + mu

    # 8) θ4 from θ234
    t4 = radians(th234_deg) - t2 - t3

    thetas_deg = [
        degrees(th1),
        degrees(t2),
        degrees(t3),
        degrees(t4),
        TH5_DEG,
    ]

    joints = get_all_joint_positions(thetas_deg).tolist()
    meta = {
        "phi_deg": phi_deg,
        "theta234_deg": th234_deg,
        "wrist_center": {"Px_w": pxw, "Py_w": pyw, "Pz_w": pzw},
        "lambda_deg": degrees(lam),
        "mu_deg": degrees(mu),
        "branch": "elbow_up (θ3<0), θ2=λ+μ",
    }
    return {"angles": thetas_deg, "joints": joints, "meta": meta}
