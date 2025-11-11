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
- **Dual Solutions**: Automatic calculation of both elbow-up and elbow-down configurations
- **Web Interface**: User-friendly Dash application with Bootstrap styling

## 🎯 Features

- ✅ Direct implementation of D-H parameters from research paper
- ✅ Real-time 3D visualization of robot configuration
- ✅ Automatic elbow configuration selection (paper-based heuristic)
- ✅ Manual elbow mode override (up/down)
- ✅ Forward kinematics validation
- ✅ Cartesian position display for all joints
- ✅ Error handling for unreachable targets

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

2. **Select Elbow Configuration**
   - **Auto (Paper-based)**: Automatically selects based on target position
     - Positive Px → Elbow-up
     - Negative Px → Elbow-down
   - **Elbow Up**: Force upper configuration
   - **Elbow Down**: Force lower configuration

3. **Calculate IK**
   - Click "Calculate Inverse Kinematics"
   - View results in:
     - Joint Angles table
     - Cartesian Configuration table
     - 3D visualization

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
   Px_w = Px - d₅·cos(φ)·cos(θ₁)
   Py_w = Py - d₅·cos(φ)·sin(θ₁)
   Pz_w = Pz + d₅·sin(φ)
   ```

3. **Elbow Configuration** (θ₂, θ₃):
   - **Elbow-up**: θ₂ = λ + μ, θ₃ = -acos(...)
   - **Elbow-down**: θ₂ = λ - μ, θ₃ = +acos(...)

4. **Wrist Pitch** (θ₄):
   ```
   θ₄ = θ₂₃₄ - θ₂ - θ₃
   ```

### Key Assumptions

Based on paper's Case 1 analysis:
- Pitch angle (φ) = 11°
- Sum constraint: θ₂ + θ₃ + θ₄ = 79°
- Roll angle (θ₅) = 90° (default)

## 🐛 Known Issues & Solutions

### Issue: Results differ from paper for Case 2
**Solution**: Use "Elbow Down" mode for negative X positions

### Issue: "Target unreachable" error
**Causes**:
- Position outside workspace (R > a₂ + a₃)
- Position too close to base (R < |a₂ - a₃|)
- Invalid Z height

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
   
2. Denavit, J., & Hartenberg, R. S. (1955). "A kinematic notation for lower-pair mechanisms based on matrices." *Journal of Applied Mechanics*, 22(2), 215-221.

## 👤 Author

**Ahmed Majid**
- GitHub: [@ahmedmajid92](https://github.com/ahmedmajid92)

## 🙏 Acknowledgments

- Original research paper authors for the mathematical foundation
- Plotly team for excellent 3D visualization tools
- Dash community for the web framework

## 📧 Contact

For questions or feedback, please open an issue on GitHub or contact me directly.

---

**⭐ If you find this project useful, please consider giving it a star!**