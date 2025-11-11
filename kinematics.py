"""
kinematics.py
Implements the "glass-box" kinematic functions for the 5-DOF robot arm
described in "Inverse Kinematics Analysis and Simulation of a 5 DOF 
Robotic Arm using MATLAB" (Al-Khwarizmi Engineering Journal, Vol. 16, No. 1, 2020).

- D-H Parameters (Table 1)
- Forward Kinematics (Eq. 1-7)
- Inverse Kinematics (Eq. 8-23)

All calculations use 'numpy' for direct replication of the paper's mathematics.
"""

import numpy as np

# --- 1. KINEMATIC MODEL DEFINITION (From Paper Table 1)  ---

# Link lengths (a_i) and offsets (d_i) in mm
a2 = 105.0
a3 = 100.0
d1 = 105.0
d5 = 150.0

def get_dh_table(thetas_deg):
    """
    Returns the D-H parameter table for the given joint angles.
    Converts angles to radians internally.
    
    Args:
        thetas_deg (list): List of 5 joint angles [t1, t2, t3, t4, t5] in degrees.
        
    Returns:
        numpy.ndarray: The 5x4 D-H parameter table.
    """
    # Convert all angles to radians
    t1, t2, t3, t4, t5 = [np.deg2rad(t) for t in thetas_deg]
    
    # D-H Parameters: [theta, d, a, alpha]
    dh_table = np.array([
        [t1, d1,   0,   np.deg2rad(90)],  # Link 1
        [t2,  0,  a2,   np.deg2rad(0)],   # Link 2
        [t3,  0,  a3,   np.deg2rad(0)],   # Link 3
        [t4,  0,   0,   np.deg2rad(90)],  # Link 4
        [t5, d5,   0,   np.deg2rad(0)]    # Link 5
    ])
    return dh_table

# --- 2. FORWARD KINEMATICS (FK) ENGINE ---

def dh_transformation_matrix(theta, d, a, alpha):
    """
    Calculates the homogeneous transformation matrix (A_i) for a single link
    using the Denavit-Hartenberg (D-H) convention.
    This is a direct implementation of the paper's Equation 1.
    
    Args:
        theta (float): Joint angle (radians)
        d (float): Link offset (mm)
        a (float): Link length (mm)
        alpha (float): Link twist (radians)
        
    Returns:
        numpy.ndarray: The 4x4 homogeneous transformation matrix.
    """
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    cos_a = np.cos(alpha)
    sin_a = np.sin(alpha)
    
    A = np.array([
        [cos_t, -sin_t * cos_a,  sin_t * sin_a,  a * cos_t],
        [sin_t,  cos_t * cos_a, -cos_t * sin_a,  a * sin_t],
        [    0,          sin_a,          cos_a,          d],
        [    0,              0,              0,          1]
    ])
    return A

def get_all_joint_positions(thetas_deg):
    """
    Calculates the Forward Kinematics for the *entire* arm.
    This function replicates the "Cartesian Configuration of Each Joint"
    output from the MATLAB GUI.
    
    It returns the 3D (x, y, z) coordinates of all 6 frames (0 to 5).
    
    Args:
        thetas_deg (list): List of 5 joint angles [t1, t2, t3, t4, t5] in degrees.
        
    Returns:
        numpy.ndarray: A 6x3 array where each row is the [x, y, z]
                       coordinate of a joint (Frame 0 to Frame 5).
    """
    dh_table = get_dh_table(thetas_deg)
    
    T_global = np.identity(4)
    positions = [[0, 0, 0]]  # Start with origin (0, 0, 0)
    
    # Calculate global transformation for each joint
    # T_0_i = A_1 * A_2 *... * A_i
    for i in range(dh_table.shape[0]):
        theta, d, a, alpha = dh_table[i]
        A_i = dh_transformation_matrix(theta, d, a, alpha)
        T_global = T_global @ A_i
        
        # Extract the position vector
        positions.append(T_global[0:3, 3].tolist())
        
    return np.array(positions)

# --- 3. INVERSE KINEMATICS (IK) ENGINE (From Paper Eq. 8-23)  ---

def calculate_ik(Px, Py, Pz, theta5_deg=90.0, elbow_mode="auto"):
    """
    Calculates the Inverse Kinematics for the 5-DOF arm based on the
    geometric solution presented in the paper.
    
    This implementation solves for (theta_1, theta_2, theta_3, theta_4)
    given a target end-effector position (Px, Py, Pz).
    
    It uses the "Hidden Assumptions" derived from the paper's
    Case 1 results (Fig. 6), as the stated equations are circular.
    1. Pitch angle phi = 11.0 degrees (Implied constant from Case 1)
    2. This implies theta_234 = 79.0 degrees (from Eq. 9: 90 - 11)
    3. Roll angle theta_5 = 90.0 degrees (from Table 2)
    
    Args:
        Px (float): Target X position (mm)
        Py (float): Target Y position (mm)
        Pz (float): Target Z position (mm)
        theta5_deg (float): Desired tool roll. Default 90, based on paper.
        elbow_mode (str): "up", "down", or "auto". 
                          "auto" selects the configuration matching the paper's cases.
        
    Returns:
        tuple: A tuple containing:
            - list: The 5 joint angles [t1, t2, t3, t4, t5] in degrees.
            - dict: A dictionary of intermediate calculation values for debugging.
    
    Raises:
        ValueError: If the target position is unreachable (e.g., outside
                    workspace, causes imaginary numbers in acos/sqrt).
    """
    
    try:
        # --- Fixed Assumptions (Based on Paper's Case 1 Results) ---
        phi_rad = np.deg2rad(11.0)
        theta_234_rad = np.deg2rad(79.0)
        
        # --- Intermediate Variable Calculations  ---
        
        # Eq. 10: R = d5 * cos(phi)
        R = d5 * np.cos(phi_rad)
        
        # Eq. 16: Solution for theta_1
        theta_1_rad = np.arctan2(Py, Px)
        
        # Eq. 11: Px_w = Px - R * cos(theta_1)
        Px_w = Px - R * np.cos(theta_1_rad)
        
        # Eq. 12: Py_w = Py - R * sin(theta_1)
        Py_w = Py - R * np.sin(theta_1_rad)
        
        # Eq. 13: Pz_w = Pz + d5 * sin(phi)
        Pz_w = Pz + d5 * np.sin(phi_rad)
        
        # Eq. 14: R_w = sqrt(Px_w^2 + Py_w^2)
        R_w = np.sqrt(Px_w**2 + Py_w**2)
        
        # Eq. 15: N = sqrt((Pz_w - d1)^2 + R_w^2)
        N = np.sqrt((Pz_w - d1)**2 + R_w**2)
        
        # --- Calculate both elbow configurations ---
        
        # Eq. 18: mu = acos( (N^2 + a2^2 - a3^2) / (2 * a2 * N) )
        cos_mu_arg = (N**2 + a2**2 - a3**2) / (2 * a2 * N)
        if not (-1 <= cos_mu_arg <= 1):
            raise ValueError(f"Target unreachable. acos(mu) arg out of range: {cos_mu_arg}")
        mu_rad = np.arccos(cos_mu_arg)
        
        # Eq. 19: lambda = atan( (Pz_w - d1) / R_w )
        lambda_rad = np.arctan2(Pz_w - d1, R_w)
        
        # Eq. 22: theta_3 argument
        cos_theta_3_arg = (N**2 - a2**2 - a3**2) / (2 * a2 * a3)
        if not (-1 <= cos_theta_3_arg <= 1):
            raise ValueError(f"Target unreachable. acos(theta_3) arg out of range: {cos_theta_3_arg}")
        
        # Calculate both configurations
        # Configuration 1: Elbow-up (lambda + mu, -acos)
        theta_2_up = lambda_rad + mu_rad
        theta_3_up = -np.arccos(cos_theta_3_arg)
        theta_4_up = theta_234_rad - theta_2_up - theta_3_up
        
        # Configuration 2: Elbow-down (lambda - mu, +acos)
        theta_2_down = lambda_rad - mu_rad
        theta_3_down = np.arccos(cos_theta_3_arg)
        theta_4_down = theta_234_rad - theta_2_down - theta_3_down
        
        # --- Select configuration based on elbow_mode ---
        
        if elbow_mode == "auto":
            # Auto-select based on paper's conventions:
            # Case 1 (positive Px, Py): elbow-up
            # Case 2 (negative Px): elbow-down
            if Px < 0:
                elbow_mode = "down"
            else:
                elbow_mode = "up"
        
        if elbow_mode == "down":
            theta_2_rad = theta_2_down
            theta_3_rad = theta_3_down
            theta_4_rad = theta_4_down
        else:  # "up"
            theta_2_rad = theta_2_up
            theta_3_rad = theta_3_up
            theta_4_rad = theta_4_up
        
        # --- Final Solution ---
        thetas_deg = [
            np.rad2deg(theta_1_rad),
            np.rad2deg(theta_2_rad),
            np.rad2deg(theta_3_rad),
            np.rad2deg(theta_4_rad),
            theta5_deg
        ]
        
        # Store intermediate values for debugging
        intermediates = {
            "phi_deg": np.rad2deg(phi_rad),
            "theta_234_deg": np.rad2deg(theta_234_rad),
            "Px_w": Px_w, "Py_w": Py_w, "Pz_w": Pz_w,
            "R_w": R_w, "N": N,
            "lambda_deg": np.rad2deg(lambda_rad),
            "mu_deg": np.rad2deg(mu_rad),
            "elbow_config": elbow_mode,
            "theta_2_up": np.rad2deg(theta_2_up),
            "theta_3_up": np.rad2deg(theta_3_up),
            "theta_4_up": np.rad2deg(theta_4_up),
            "theta_2_down": np.rad2deg(theta_2_down),
            "theta_3_down": np.rad2deg(theta_3_down),
            "theta_4_down": np.rad2deg(theta_4_down)
        }
        
        return thetas_deg, intermediates

    except Exception as e:
        # Catch any math errors (e.g., divide by zero, sqrt of negative)
        raise ValueError(f"IK calculation error: {e}")

def calculate_ik_both_solutions(Px, Py, Pz, theta5_deg=90.0, tolerance=0.5):
    """
    Calculates both elbow configurations and validates them via forward kinematics.
    
    Args:
        Px (float): Target X position (mm)
        Py (float): Target Y position (mm)
        Pz (float): Target Z position (mm)
        theta5_deg (float): Desired tool roll. Default 90.
        tolerance (float): Maximum position error in mm for FK validation.
        
    Returns:
        dict: Dictionary with "elbow_up" and "elbow_down" solutions,
              each containing angles and FK validation results.
    """
    solutions = {}
    
    for mode in ["up", "down"]:
        try:
            thetas_deg, intermediates = calculate_ik(Px, Py, Pz, theta5_deg, elbow_mode=mode)
            
            # Validate with forward kinematics
            joint_positions = get_all_joint_positions(thetas_deg)
            end_effector_pos = joint_positions[-1]  # Last joint (Frame 5)
            
            # Calculate position error
            error = np.sqrt((end_effector_pos[0] - Px)**2 + 
                          (end_effector_pos[1] - Py)**2 + 
                          (end_effector_pos[2] - Pz)**2)
            
            solutions[f"elbow_{mode}"] = {
                "angles": thetas_deg,
                "fk_position": end_effector_pos.tolist(),
                "position_error": error,
                "valid": error < tolerance,
                "intermediates": intermediates
            }
        except ValueError as e:
            solutions[f"elbow_{mode}"] = {
                "error": str(e),
                "valid": False
            }
    
    return solutions