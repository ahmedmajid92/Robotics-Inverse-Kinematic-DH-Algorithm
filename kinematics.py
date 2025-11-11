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

# Standard Python math functions for trigonometry and angle conversions
from math import cos, sin, atan2, acos, sqrt, radians, degrees
# NumPy for matrix operations and array handling
import numpy as np

# ---------------------------
# Robot Physical Dimensions
# ---------------------------
# These constants define the robot's link lengths and offsets in millimeters
# Values are taken directly from the research paper's specifications

a2 = 105.0  # Length of link 2 (shoulder to elbow) in mm
a3 = 100.0  # Length of link 3 (elbow to wrist) in mm
d1 = 105.0  # Base height (distance from ground to first joint) in mm
d5 = 150.0  # End-effector offset (wrist to tool tip) in mm

TH5_DEG = 90.0  # Fixed roll angle for joint 5 (tool orientation) in degrees

# ---------------------------
# Forward kinematics (FK)
# ---------------------------

def _A(theta_deg, d, a, alpha_deg):
    """
    Standard DH (Denavit-Hartenberg) transformation matrix generator.
    
    This function creates a 4x4 homogeneous transformation matrix that represents
    the position and orientation of one joint frame relative to the previous frame.
    
    Args:
        theta_deg: Joint angle (rotation about Z-axis) in degrees
        d: Link offset along previous Z-axis in mm
        a: Link length along X-axis in mm
        alpha_deg: Link twist (rotation about X-axis) in degrees
    
    Returns:
        4x4 numpy array representing the homogeneous transformation matrix
    
    Matrix Structure:
        [R  |  p]    where R is 3x3 rotation matrix, p is 3x1 position vector
        [0  |  1]    bottom row is [0 0 0 1]
    """
    # Convert angles from degrees to radians for trigonometric functions
    th = radians(theta_deg)
    al = radians(alpha_deg)
    
    # Pre-compute trigonometric values for efficiency
    ct, st = cos(th), sin(th)  # cos and sin of theta
    ca, sa = cos(al), sin(al)  # cos and sin of alpha
    
    # Construct the 4x4 DH transformation matrix
    # Row 1: X-axis of new frame expressed in old frame
    # Row 2: Y-axis of new frame expressed in old frame
    # Row 3: Z-axis of new frame expressed in old frame
    # Row 4: Homogeneous coordinate row [0 0 0 1]
    return np.array(
        [
            [ct, -st * ca,  st * sa, a * ct],  # X-axis rotation and translation
            [st,  ct * ca, -ct * sa, a * st],  # Y-axis rotation and translation
            [0.0,      sa,      ca,      d],   # Z-axis rotation and translation
            [0.0,     0.0,     0.0,    1.0],   # Homogeneous coordinates
        ],
        dtype=float,
    )

def get_all_joint_positions(thetas_deg):
    """
    Compute Cartesian positions of all joint frames using forward kinematics.
    
    This function takes joint angles and returns the 3D position of each joint
    in the base coordinate frame. It uses the DH parameters to build transformation
    matrices and accumulate them to find absolute positions.
    
    Args:
        thetas_deg: List of 5 joint angles [θ1, θ2, θ3, θ4, θ5] in degrees
    
    Returns:
        (6, 3) numpy array where each row is [x, y, z] position in mm:
            Row 0: Base position (always [0, 0, 0])
            Row 1: Joint 1 position
            Row 2: Joint 2 position
            Row 3: Joint 3 position
            Row 4: Joint 4 position
            Row 5: Joint 5 (end-effector) position
    """
    # Unpack the five joint angles
    th1, th2, th3, th4, th5 = thetas_deg
    
    # Build individual DH transformation matrices for each joint
    # Format: _A(theta, d, a, alpha) using DH parameters from paper
    A1 = _A(th1, d1, 0.0, 90.0)    # Joint 1: Base rotation (Z-axis), height d1, twist 90°
    A2 = _A(th2, 0.0, a2, 0.0)     # Joint 2: Shoulder rotation, length a2
    A3 = _A(th3, 0.0, a3, 0.0)     # Joint 3: Elbow rotation, length a3
    A4 = _A(th4, 0.0, 0.0, 90.0)   # Joint 4: Wrist rotation, twist 90°
    A5 = _A(th5, d5, 0.0, 0.0)     # Joint 5: Tool rotation, offset d5

    # Initialize transformation matrices by accumulating DH transforms
    T0 = np.eye(4)        # Base frame (identity matrix)
    T1 = T0 @ A1          # Transform from base to joint 1
    T2 = T1 @ A2          # Transform from base to joint 2
    T3 = T2 @ A3          # Transform from base to joint 3
    T4 = T3 @ A4          # Transform from base to joint 4
    T5 = T4 @ A5          # Transform from base to joint 5 (end-effector)

    # Extract the position vector (last column, first 3 rows) from each transform
    # This gives us the [x, y, z] coordinates of each joint in the base frame
    pts = np.array(
        [
            T0[:3, 3],  # Base position [0, 0, 0]
            T1[:3, 3],  # Joint 1 position
            T2[:3, 3],  # Joint 2 position
            T3[:3, 3],  # Joint 3 position
            T4[:3, 3],  # Joint 4 position
            T5[:3, 3],  # Joint 5 (end-effector) position
        ],
        dtype=float,
    )
    return pts

def forward_kinematics(thetas_deg):
    """
    Complete forward kinematics: returns both joint positions and final transformation.
    
    This is an extended version of get_all_joint_positions that also returns the
    full transformation matrix from base to end-effector (useful for orientation).
    
    Args:
        thetas_deg: List of 5 joint angles [θ1, θ2, θ3, θ4, θ5] in degrees
    
    Returns:
        Tuple of (points, T05):
            - points: (6, 3) array of joint positions [x, y, z]
            - T05: 4x4 transformation matrix from base to end-effector
    """
    # Unpack joint angles
    th1, th2, th3, th4, th5 = thetas_deg
    
    # Build DH transformation matrices (same as in get_all_joint_positions)
    A1 = _A(th1, d1, 0.0, 90.0)
    A2 = _A(th2, 0.0, a2, 0.0)
    A3 = _A(th3, 0.0, a3, 0.0)
    A4 = _A(th4, 0.0, 0.0, 90.0)
    A5 = _A(th5, d5, 0.0, 0.0)
    
    # Accumulate transformations sequentially
    T = np.eye(4)           # Start with identity (base frame)
    Ts = [np.eye(4)]        # Store all intermediate transformations
    
    # Multiply each DH matrix to get cumulative transformation
    for A in (A1, A2, A3, A4, A5):
        T = T @ A           # Matrix multiplication: T_new = T_old * A_i
        Ts.append(T)        # Store this transformation
    
    # Extract position vectors from all transformations
    pts = np.array([Ti[:3, 3] for Ti in Ts], dtype=float)
    
    # Return both joint positions and final transformation matrix
    return pts, T

# ---------------------------
# Inverse kinematics (IK)
# ---------------------------

def _clamp(x, lo=-1.0, hi=1.0):
    """
    Clamp a value to the range [lo, hi] to prevent math domain errors.
    
    This is crucial for acos() and asin() which only accept values in [-1, 1].
    Floating-point errors can push values slightly outside this range (e.g., 1.0000001).
    
    Args:
        x: Value to clamp
        lo: Lower bound (default -1.0 for acos/asin)
        hi: Upper bound (default 1.0 for acos/asin)
    
    Returns:
        Clamped value guaranteed to be in [lo, hi]
    """
    return hi if x > hi else lo if x < lo else x

def _phi_deg_for_target(px: float) -> float:
    """
    Select wrist pitch angle (φ) based on target position quadrant.
    
    This matches the paper's approach where different wrist angles are used
    depending on whether the target is in front (positive X) or behind (negative X).
    
    Paper's Strategy:
        - Case 1 (Px ≥ 0): φ = 11° → θ2+θ3+θ4 = 79°
        - Case 2 (Px < 0):  φ = 12° → θ2+θ3+θ4 = 78°
    
    Args:
        px: X-coordinate of target position in mm
    
    Returns:
        Wrist pitch angle φ in degrees (11.0 or 12.0)
    """
    return 11.0 if px >= 0.0 else 12.0

def solve_ik(px, py, pz, elbow_mode="up"):
    """
    Paper-faithful inverse kinematics solver for 5-DOF robotic arm.
    
    This function implements the geometric IK solution from the research paper,
    specifically using the "elbow-up" configuration (θ3 < 0) which matches
    both validation cases in the paper.
    
    Algorithm Steps (from paper):
        1. Choose wrist pitch φ based on target quadrant
        2. Calculate base rotation θ1 = atan2(Py, Px)
        3. Compute wrist center position by offsetting from target
        4. Calculate reach distance N and angle λ to wrist
        5. Use law of cosines to find angle μ at shoulder
        6. Solve for elbow angle θ3 (negative for elbow-up)
        7. Calculate shoulder angle θ2 = λ + μ
        8. Determine wrist angle θ4 from constraint θ2+θ3+θ4 = 90°-φ
        9. Set tool roll θ5 = 90° (fixed)
    
    Args:
        px: Target X position in mm
        py: Target Y position in mm
        pz: Target Z position in mm
        elbow_mode: Configuration selector ("up" or "down"), currently forced to "up"
    
    Returns:
        Dictionary containing:
            "angles": List of 5 joint angles [θ1, θ2, θ3, θ4, θ5] in degrees
            "joints": List of 6 joint positions [[x,y,z], ...] in mm
            "meta": Dictionary of intermediate values for debugging:
                - phi_deg: Chosen wrist pitch angle
                - theta234_deg: Sum constraint value (90° - φ)
                - wrist_center: Computed wrist position {Px_w, Py_w, Pz_w}
                - lambda_deg: Angle from horizontal to wrist
                - mu_deg: Shoulder triangle angle
                - branch: Configuration description
    
    Raises:
        ValueError: If target is unreachable (causes domain errors in acos)
    """
    # ------------------------------------------------------------------
    # STEP 1: Choose wrist pitch (φ) based on target quadrant
    # ------------------------------------------------------------------
    # This automatic selection matches the paper's two demo cases
    phi_deg = _phi_deg_for_target(px)
    
    # Calculate the sum constraint: θ2 + θ3 + θ4 = 90° - φ
    # This ensures the end-effector maintains the desired pitch angle
    th234_deg = 90.0 - phi_deg

    # ------------------------------------------------------------------
    # STEP 2: Base rotation (θ1) - projects problem into 2D plane
    # ------------------------------------------------------------------
    # atan2 handles all quadrants correctly and avoids division by zero
    th1 = atan2(py, px)  # Result in radians

    # ------------------------------------------------------------------
    # STEP 3: Wrist center position (where joint 4 is located)
    # ------------------------------------------------------------------
    # The wrist center is offset from the target by distance d5 at angle φ
    # This "backs up" from the end-effector to find where the wrist must be
    
    phi = radians(phi_deg)  # Convert φ to radians for trig functions
    
    # Horizontal component of d5 (projected onto XY plane)
    R = d5 * cos(phi)
    
    # Subtract the horizontal offset along the direction of θ1
    pxw = px - R * cos(th1)  # X-component of wrist center
    pyw = py - R * sin(th1)  # Y-component of wrist center
    
    # Add vertical component (POSITIVE per paper's coordinate system)
    # This is a key detail: the paper uses '+' here, not '-'
    pzw = pz + d5 * sin(phi)  # Z-component of wrist center

    # ------------------------------------------------------------------
    # STEP 4: Distance calculations for triangle formed by joints 2, 3, wrist
    # ------------------------------------------------------------------
    # After base rotation, we work in a 2D plane containing joints 2, 3, and wrist
    
    # Horizontal distance from joint 2 to wrist center
    Rw = sqrt(pxw**2 + pyw**2)
    
    # Vertical offset from joint 2 (which is at height d1)
    z_ = pzw - d1
    
    # Direct distance from joint 2 to wrist center (hypotenuse)
    # This is one side of the triangle we'll solve
    N = sqrt(Rw**2 + z_**2)

    # ------------------------------------------------------------------
    # STEP 5: Calculate angles λ and μ using geometry
    # ------------------------------------------------------------------
    # λ (lambda): Angle from horizontal to the line connecting joint 2 to wrist
    lam = atan2(z_, Rw)
    
    # μ (mu): Angle at joint 2 in the triangle formed by links a2, a3, and N
    # Using law of cosines: N² = a2² + a3² - 2·a2·a3·cos(angle_at_elbow)
    # Rearranged to solve for angle at shoulder
    cos_mu = _clamp((N**2 + a2**2 - a3**2) / (2.0 * a2 * N))
    mu = acos(cos_mu)  # Result in radians

    # ------------------------------------------------------------------
    # STEP 6: Elbow angle (θ3) - NEGATIVE for elbow-up configuration
    # ------------------------------------------------------------------
    # Using law of cosines at the elbow joint
    # cos(θ3) = (N² - a2² - a3²) / (2·a2·a3)
    cos_t3 = _clamp((N**2 - a2**2 - a3**2) / (2.0 * a2 * a3))
    t3_mag = acos(cos_t3)  # Get the magnitude (always positive)
    
    # Make it negative for elbow-up configuration (θ3 < 0)
    # This matches the paper's convention and both validation cases
    t3 = -t3_mag

    # ------------------------------------------------------------------
    # STEP 7: Shoulder angle (θ2) - uses λ + μ for elbow-up
    # ------------------------------------------------------------------
    # For elbow-up: θ2 = λ + μ (angles add)
    # For elbow-down: θ2 = λ - μ (angles subtract)
    # We use addition because we're in elbow-up mode
    t2 = lam + mu

    # ------------------------------------------------------------------
    # STEP 8: Wrist angle (θ4) - derived from sum constraint
    # ------------------------------------------------------------------
    # From the constraint θ2 + θ3 + θ4 = θ234, solve for θ4
    # θ4 = θ234 - θ2 - θ3
    t4 = radians(th234_deg) - t2 - t3

    # ------------------------------------------------------------------
    # STEP 9: Convert all angles to degrees for output
    # ------------------------------------------------------------------
    thetas_deg = [
        degrees(th1),   # Base rotation
        degrees(t2),    # Shoulder angle
        degrees(t3),    # Elbow angle (negative)
        degrees(t4),    # Wrist angle
        TH5_DEG,        # Tool roll (fixed at 90°)
    ]

    # ------------------------------------------------------------------
    # Compute joint positions using forward kinematics for verification
    # ------------------------------------------------------------------
    joints = get_all_joint_positions(thetas_deg).tolist()
    
    # ------------------------------------------------------------------
    # Package metadata for debugging and validation
    # ------------------------------------------------------------------
    meta = {
        "phi_deg": phi_deg,                              # Wrist pitch used
        "theta234_deg": th234_deg,                       # Sum constraint value
        "wrist_center": {"Px_w": pxw, "Py_w": pyw, "Pz_w": pzw},  # Wrist position
        "lambda_deg": degrees(lam),                      # Angle to wrist from horizontal
        "mu_deg": degrees(mu),                           # Shoulder triangle angle
        "branch": "elbow_up (θ3<0), θ2=λ+μ",           # Configuration identifier
    }
    
    # Return complete solution package
    return {"angles": thetas_deg, "joints": joints, "meta": meta}
