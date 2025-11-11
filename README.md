# 5-DOF Robotic Arm - Inverse Kinematics Simulator

A web-based interactive simulation tool for analyzing and visualizing the inverse kinematics of a 5-degree-of-freedom (5-DOF) robotic arm using the Denavit-Hartenberg (D-H) convention.

## 📋 Overview

This project implements the inverse kinematics solution described in the research paper:
> **"Inverse Kinematics Analysis and Simulation of a 5 DOF Robotic Arm using MATLAB"**  
> *Al-Khwarizmi Engineering Journal, Vol. 16, No. 1, 2020*

The application provides:
- **Forward Kinematics (FK)**: Calculate joint positions from joint angles
- **Inverse Kinematics (IK)**: Calculate joint angles from desired end-effector position
- **3D Visualization**: Interactive Plotly-based 3D rendering of the robot arm
- **Paper-Faithful Implementation**: Elbow-up configuration matching research paper results
- **Web Interface**: User-friendly Dash application with Bootstrap styling

## 🎯 Features

- ✅ Direct implementation of D-H parameters from research paper
- ✅ Real-time 3D visualization of robot configuration
- ✅ Elbow-up configuration (matching paper's approach)
- ✅ Forward kinematics validation
- ✅ Cartesian position display for all joints (0-5)
- ✅ Error handling for unreachable targets
- ✅ Automatic wrist pitch selection based on target position

## 🛠️ Technologies

- **Python 3.10**
- **NumPy**: Mathematical computations
- **Plotly**: 3D graphics and visualization
- **Dash**: Web application framework
- **Dash Bootstrap Components**: UI styling

## 📦 Installation

### Prerequisites
- [Anaconda](https://www.anaconda.com/products/distribution) or [Miniconda](https://docs.conda.io/en/latest/miniconda.html)
- Git

### Setup

1. **Clone the repository**
   ```bash
   git clone git@github.com:ahmedmajid92/Robotics-Inverse-Kinematic-DH-Algorithm.git
   cd Robotics-Inverse-Kinematic-DH-Algorithm
   ```

2. **Create Conda environment**
   ```bash
   conda env create -f environment.yml
   ```

3. **Activate the environment**
   ```bash
   conda activate robotics_ik
   ```

4. **Run the application**
   ```bash
   python app.py
   ```

5. **Open your browser**
   - Navigate to: `http://127.0.0.1:8050/`

## 🎮 Usage

### Basic Operation

1. **Enter Target Position**
   - Input desired X, Y, Z coordinates (in millimeters)
   - Example inputs from paper:
     - Case 1: `(220, 161, 220)`
     - Case 2: `(-230, 61, 220)`

2. **Elbow Configuration**
   - The application uses **Elbow Up** configuration by default
   - This matches the approach used in the research paper
   - Dropdown is available but defaults to "Elbow Up" for consistency

3. **Calculate IK**
   - Click "Calculate Inverse Kinematics"
   - View results in:
     - Joint Angles table (θ₁ through θ₅)
     - Cartesian Configuration table (Positions of Joints 1-5)
     - 3D visualization with proper axis orientation

### Robot Specifications

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Link 2 Length | a₂ | 105 | mm |
| Link 3 Length | a₃ | 100 | mm |
| Base Height | d₁ | 105 | mm |
| End-Effector Offset | d₅ | 150 | mm |

### D-H Parameters

| Joint | θᵢ | dᵢ | aᵢ | αᵢ |
|-------|-----|-----|-----|-----|
| 1 | θ₁* | 105 | 0 | 90° |
| 2 | θ₂* | 0 | 105 | 0° |
| 3 | θ₃* | 0 | 100 | 0° |
| 4 | θ₄* | 0 | 0 | 90° |
| 5 | θ₅* | 150 | 0 | 0° |

*Variable joint angles

## 📁 Project Structure

```
Robotics_IK_Project/
│
├── app.py                 # Main Dash web application
├── kinematics.py          # IK/FK algorithms and D-H computations
├── environment.yml        # Conda environment specification
├── README.md             # This file
├── .gitignore            # Git ignore rules
└── zmosa,+1-10.pdf       # Reference research paper
```

## 🔬 Algorithm Details

### Inverse Kinematics Solution

The IK solution uses a geometric approach with the following key equations:

1. **Base Rotation** (θ₁):
   ```
   θ₁ = atan2(Py, Px)
   ```

2. **Wrist Position Calculation**:
   ```
   R = d₅ · cos(φ)
   Px_w = Px - R · cos(θ₁)
   Py_w = Py - R · sin(θ₁)
   Pz_w = Pz + d₅ · sin(φ)
   ```

3. **Elbow Configuration** (θ₂, θ₃):
   - **Elbow-up** (default): 
     - θ₃ = -acos((N² - a₂² - a₃²)/(2·a₂·a₃))
     - θ₂ = λ + μ

4. **Wrist Pitch** (θ₄):
   ```
   θ₄ = (90° - φ) - θ₂ - θ₃
   ```

### Key Assumptions (Paper-Faithful)

- **Wrist Pitch Selection**:
  - Px ≥ 0 → φ = 11° (θ₂₃₄ = 79°) [Case 1]
  - Px < 0 → φ = 12° (θ₂₃₄ = 78°) [Case 2]
- **Roll Angle**: θ₅ = 90° (fixed)
- **Elbow Configuration**: Elbow-up (θ₃ < 0)

## 🎨 Visualization Details

The 3D plot uses the following coordinate system:
- **X-axis**: Increases from left to right (range: [-400, 400] mm)
- **Y-axis**: Increases from right to left (range: [-400, 400] mm)
- **Z-axis**: Vertical axis (range: [-400, 400] mm)
- **Red baseline**: Shows base connection from origin to z = -400 mm
- **Camera position**: Optimized for clear viewing (eye: x=2.2, y=0.8, z=1.4)

## 🧮 Test Cases

### Case 1: Positive Quadrant
```
Input: (220, 161, 220)
Expected Output:
θ₁ ≈ 36.1°
θ₂ ≈ 79.5°
θ₃ ≈ -56.3°
θ₄ ≈ 55.8°
θ₅ = 90.0°
```

### Case 2: Negative X
```
Input: (-230, 61, 220)
Expected Output:
θ₁ ≈ 165.1°
θ₂ ≈ 90.8°
θ₃ ≈ -68.1°
θ₄ ≈ 56.3°
θ₅ = 90.0°
```

## 🐛 Known Issues & Solutions

### Issue: Target unreachable error
**Causes**:
- Position outside workspace (√(Px² + Py²) > a₂ + a₃ ≈ 205 mm)
- Position too close to base (√(Px² + Py²) < |a₂ - a₃| ≈ 5 mm)
- Invalid Z height (Z < d₁ or Z > d₁ + a₂ + a₃)

**Solution**: Verify target coordinates are within reachable workspace

### Issue: Math domain errors
**Causes**:
- acos() arguments outside [-1, 1] range

**Solution**: The code includes `_clamp()` function to prevent this

## 🤝 Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📚 References

1. Al-Khwarizmi Engineering Journal, Vol. 16, No. 1, 2020
   - "Inverse Kinematics Analysis and Simulation of a 5 DOF Robotic Arm using MATLAB"
   - DOI: Available in paper
   
2. Denavit, J., & Hartenberg, R. S. (1955). 
   - "A kinematic notation for lower-pair mechanisms based on matrices." 
   - *Journal of Applied Mechanics*, 22(2), 215-221.

## 👤 Author

**Ahmed Majid**
- GitHub: [@ahmedmajid92](https://github.com/ahmedmajid92)

## 🙏 Acknowledgments

- Original research paper authors for the mathematical foundation and validation cases
- Plotly team for excellent 3D visualization capabilities
- Dash community for the reactive web framework
- NumPy developers for robust numerical computations

## 📧 Contact

For questions or feedback:
- Open an issue on GitHub
- Email: [Contact via GitHub profile]

## 🔄 Version History

### v1.0.0 (Current)
- Initial release
- Paper-faithful implementation with elbow-up configuration
- Automatic wrist pitch selection based on target position
- 3D visualization with proper coordinate system
- Error handling for unreachable targets

---

**⭐ If you find this project useful, please consider giving it a star!**
