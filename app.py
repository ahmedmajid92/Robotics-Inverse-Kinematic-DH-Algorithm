"""
app.py
Creates an interactive web application (Plotly Dash) to control and 
visualize the 5-DOF robotic arm.

This application replicates the functionality of the MATLAB GUI shown 
in the paper, providing:
    - User input fields for target position (Px, Py, Pz)
    - Elbow configuration selector (elbow-up or elbow-down)
    - 3D visualization of robot arm using Plotly
    - Output tables showing joint angles and positions
    - Error handling with fallback to home position
"""

# Core Dash framework imports
import dash
from dash import dcc, html, dash_table, Input, Output, State, clientside_callback

# Plotly for 3D visualization
import plotly.graph_objects as go

# Bootstrap components for professional UI layout
import dash_bootstrap_components as dbc

# NumPy for array operations
import numpy as np

# Import our custom kinematics module (IK/FK solvers)
import kinematics as kin

# ==============================================================================
# VISUALIZATION FUNCTIONS
# ==============================================================================

def get_cylinder_mesh(p1, p2, radius, n_sides=20):
    """
    Generates mesh data for a cylinder between two points.
    """
    length = np.linalg.norm(p2 - p1)
    if length == 0:
        return [], [], [], [], [], []

    # Cylinder axis
    axis = (p2 - p1) / length

    # Perpendicular vector
    if np.abs(axis[2]) < 0.99:
        perp_axis = np.cross(axis, [0, 0, 1])
    else:
        perp_axis = np.cross(axis, [0, 1, 0])
    perp_axis /= np.linalg.norm(perp_axis)
    
    # Second perpendicular vector
    perp_axis2 = np.cross(axis, perp_axis)
    
    t = np.linspace(0, 2 * np.pi, n_sides + 1)
    
    # Points on the circle
    circ_pts = radius * (np.outer(np.cos(t), perp_axis) + np.outer(np.sin(t), perp_axis2))

    # Cylinder vertices
    p_start = p1 + circ_pts
    p_end = p2 + circ_pts

    x = np.vstack([p_start[:, 0], p_end[:, 0]]).flatten('F')
    y = np.vstack([p_start[:, 1], p_end[:, 1]]).flatten('F')
    z = np.vstack([p_start[:, 2], p_end[:, 2]]).flatten('F')
    
    # Generate indices for triangles
    # Vertices: 0..n_sides (bottom), n_sides+1 .. 2*n_sides+1 (top)
    n_p = n_sides + 1
    i_idx = []
    j_idx = []
    k_idx = []
    
    for k in range(n_sides):
        # Triangle 1
        i_idx.append(k)
        j_idx.append(k + 1)
        k_idx.append(k + n_p)
        
        # Triangle 2
        i_idx.append(k + 1)
        j_idx.append(k + n_p + 1)
        k_idx.append(k + n_p)
        
    return x, y, z, i_idx, j_idx, k_idx

def plot_robot_arm(joint_positions, fig=None):
    """
    Creates a 3D Plotly figure of the robotic arm with paper-aligned axes.
    
    This function generates an interactive 3D visualization where users can
    rotate, zoom, and pan to view the robot from different angles. The
    coordinate system is carefully configured to match the paper's conventions.
    
    Args:
        joint_positions (numpy.ndarray): A 6x3 array of [x, y, z] coordinates
            representing positions of frames 0 through 5:
                Row 0: Base (always at origin [0, 0, 0])
                Row 1: Joint 1 position (after base rotation)
                Row 2: Joint 2 position (shoulder)
                Row 3: Joint 3 position (elbow)
                Row 4: Joint 4 position (wrist)
                Row 5: Joint 5 position (end-effector/tool)
        fig (plotly.graph_objects.Figure, optional): An existing figure to update. 
            If None, a new figure is created. Defaults to None.

    Returns:
        plotly.graph_objects.Figure: Interactive 3D plot configured with:
            - Arm links as thick lines connecting joints
            - Joint markers as circles
            - Red baseline showing base height
            - Symmetric axis ranges for undistorted view
            - Camera positioned for optimal viewing angle
    
    Coordinate System Alignment:
        The plot uses axis range reversal to match the paper's visual layout:
        - X-axis: Increases left → right (range reversed to [400, -400])
        - Y-axis: Increases right → left (range reversed, data negated)
        - Z-axis: Standard vertical axis (range [-400, 400])
    """
    jp = np.asarray(joint_positions, dtype=float)

    x_plot = jp[:, 0]
    y_plot = -jp[:, 1]
    z_plot = jp[:, 2]

    if fig is None:
        fig = go.Figure()
    else:
        fig.data = []

    # Define link radii and colors
    link_radii = [15, 12, 12, 10, 8]
    link_colors = ['#E41A1C', '#377EB8', '#4DAF4A', '#984EA3', '#FF7F00'] # distinct colors

    # Add cylinders for each link
    for i in range(len(jp) - 1):
        p1 = jp[i]
        p2 = jp[i+1]
        radius = link_radii[i]
        cx, cy, cz, ci, cj, ck = get_cylinder_mesh(p1, p2, radius)
        
        # Apply visualization transform to cylinder coordinates
        cy_plot = -np.array(cy)
        
        fig.add_trace(go.Mesh3d(
            x=cx, y=cy_plot, z=cz,
            i=ci, j=cj, k=ck,
            opacity=1.0,
            color=link_colors[i],
            name=f'Link {i+1}',
            lighting=dict(ambient=0.6, diffuse=0.8, specular=0.2, roughness=0.5),
            lightposition=dict(x=100, y=100, z=1000)
        ))

    # Add spheres for joints (exclude base at index 0)
    fig.add_trace(go.Scatter3d(
        x=x_plot[1:],   # Skip base
        y=y_plot[1:],
        z=z_plot[1:],
        mode='markers',
        marker=dict(size=10, color='blue', symbol='circle'),
        name='Joints'
    ))

    # Add base as a flat cylinder (disk)
    base_radius = 60
    base_height = 2
    # Use get_cylinder_mesh to create a short cylinder for the base
    cx, cy, cz, ci, cj, ck = get_cylinder_mesh(
        np.array([0, 0, -base_height]), 
        np.array([0, 0, 0]), 
        base_radius
    )
    cy_plot = -np.array(cy)
    
    fig.add_trace(go.Mesh3d(
        x=cx, y=cy_plot, z=cz,
        i=ci, j=cj, k=ck,
        opacity=1.0,
        color='black',
        name='Base'
    ))

    # Add red baseline from base to z = -400
    fig.add_trace(go.Scatter3d(
        x=[0, 0],
        y=[0, 0],
        z=[0, -400],
        mode='lines',
        line=dict(color='red', width=6),
        name='Base Line',
        showlegend=False
    ))

    # Configure plot layout
    AX_MIN, AX_MAX = -400.0, 400.0
    fig.update_layout(
        title="3D Simulation",
        uirevision='constant', # Preserve camera view across updates
        scene=dict(
            xaxis=dict(title='X (mm)', range=[AX_MAX, AX_MIN], autorange=False),
            yaxis=dict(title='Y (mm)', range=[AX_MAX, AX_MIN], autorange=False),
            zaxis=dict(title='Z (mm)', range=[AX_MIN, AX_MAX], autorange=False),
            aspectmode='cube',
            camera=dict(
                up=dict(x=0, y=0, z=1),
                center=dict(x=0, y=0, z=0),
                eye=dict(x=2.2, y=0.8, z=1.4),
            ),
        ),
        margin=dict(l=0, r=0, b=0, t=40),
        showlegend=False
    )
    
    return fig

# ==============================================================================
# TABLE FORMATTING FUNCTIONS
# ==============================================================================

def create_angles_table(thetas_deg):
    """
    Format joint angles into a list of dictionaries for Dash DataTable.
    
    This function converts the raw angle values into a structured format
    that can be displayed in a table with proper labels and formatting.
    
    Args:
        thetas_deg: List of 5 joint angles [θ1, θ2, θ3, θ4, θ5] in degrees
    
    Returns:
        List of dictionaries, each containing:
            'Joint': Joint name (e.g., "Theta 1")
            'Angle (deg)': Angle value formatted to 4 decimal places
    
    Example Output:
        [
            {'Joint': 'Theta 1', 'Angle (deg)': '36.1234'},
            {'Joint': 'Theta 2', 'Angle (deg)': '79.5678'},
            ...
        ]
    """
    return [
        {'Joint': 'Theta 1', 'Angle (deg)': f"{thetas_deg[0]:.4f}"},  # Base rotation
        {'Joint': 'Theta 2', 'Angle (deg)': f"{thetas_deg[1]:.4f}"},  # Shoulder
        {'Joint': 'Theta 3', 'Angle (deg)': f"{thetas_deg[2]:.4f}"},  # Elbow
        {'Joint': 'Theta 4', 'Angle (deg)': f"{thetas_deg[3]:.4f}"},  # Wrist
        {'Joint': 'Theta 5', 'Angle (deg)': f"{thetas_deg[4]:.4f}"},  # Tool roll
    ]

def create_positions_table(joint_positions):
    """
    Format joint positions into a list of dictionaries for Dash DataTable.
    
    This function displays the Cartesian coordinates of each joint in the
    base frame. Note: We show the RAW coordinate values here (no Y-flip)
    so the table reflects the actual mathematical model, not the visualization.
    
    Args:
        joint_positions: (6, 3) numpy array of [x, y, z] positions in mm
    
    Returns:
        List of dictionaries for joints 1-5, each containing:
            'Joint': Joint description
            'Px (mm)': X coordinate (4 decimal places)
            'Py (mm)': Y coordinate (4 decimal places)
            'Pz (mm)': Z coordinate (4 decimal places)
    
    Note:
        We start from index 1 (skip base at index 0) and show joints 1-5.
        The Y values here are NOT negated (unlike in the 3D plot) because
        this table shows the mathematical model coordinates, not visual coords.
    """
    # Convert to numpy array and ensure float type
    jp = np.asarray(joint_positions, dtype=float)
    
    # Build list of dictionaries, one per joint
    rows = []
    for i in range(1, 6):  # Joints 1 through 5 (skip base at index 0)
        x, y, z = jp[i]    # Extract coordinates
        rows.append({
            'Joint': f'Position at Joint {i}',  # Label like "Position at Joint 1"
            'Px (mm)': f"{x:.4f}",              # X coordinate, 4 decimal places
            'Py (mm)': f"{y:.4f}",              # Y coordinate (RAW, not negated)
            'Pz (mm)': f"{z:.4f}",              # Z coordinate
        })
    
    return rows

# ==============================================================================
# DASH APPLICATION SETUP
# ==============================================================================

# Initialize Dash app with Bootstrap theme for professional styling
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.LUX])

# Expose Flask server for deployment (e.g., to Heroku, AWS)
server = app.server

# Set browser tab title
app.title = "5-DOF Robot IK"

# ==============================================================================
# LAYOUT COMPONENTS
# ==============================================================================

# ------------------------------------------------------------------
# Define styles for tables (consistent appearance across app)
# ------------------------------------------------------------------
# Blue header to match Bootstrap primary color scheme
table_header_style = {
    "backgroundColor": "#007BFF",    # Primary blue
    "color": "white",                # White text for contrast
    "fontWeight": "bold"             # Bold for emphasis
}

# Monospace font for numeric data (easier to read aligned numbers)
table_cell_style = {
    "textAlign": "left",                      # Left-align text
    "fontFamily": "Consolas, monospace"       # Monospace font for numbers
}

# ------------------------------------------------------------------
# Input Card: Contains target position inputs and solve button
# ------------------------------------------------------------------
inputs_card = dbc.Card([
    dbc.CardBody([
        # Card title
        html.H5("Target End-Effector Position", className="card-title"),
        
        # Px input row (X coordinate)
        dbc.Row([
            dbc.Col(dbc.Label("Px:"), width=3, className="text-end"),  # Right-aligned label
            dbc.Col(
                dcc.Input(
                    id='px-input',           # Unique ID for callback
                    type='number',           # Numeric input with +/- buttons
                    value=220.0,             # Default from paper's Case 1
                    step=0.1,                # Increment by 0.1 mm per click
                    className="w-100"        # Full width of column
                ),
                width=9                      # 9/12 columns for input
            ),
        ], className="mb-2 align-items-center"),  # Margin bottom, vertically centered
        
        # Py input row (Y coordinate)
        dbc.Row([
            dbc.Col(dbc.Label("Py:"), width=3, className="text-end"),
            dbc.Col(
                dcc.Input(
                    id='py-input',
                    type='number',
                    value=161.0,             # Default from paper's Case 1
                    step=0.1,
                    className="w-100"
                ),
                width=9
            ),
        ], className="mb-2 align-items-center"),
        
        # Pz input row (Z coordinate)
        dbc.Row([
            dbc.Col(dbc.Label("Pz:"), width=3, className="text-end"),
            dbc.Col(
                dcc.Input(
                    id='pz-input',
                    type='number',
                    value=220.0,             # Default from paper's Case 1
                    step=0.1,
                    className="w-100"
                ),
                width=9
            ),
        ], className="mb-2 align-items-center"),
        
        # Horizontal rule separator
        html.Hr(),
        
        # Elbow configuration dropdown (now functional with 2 options)
        dbc.Row([
            dbc.Col(dbc.Label("Elbow Config:"), width=3, className="text-end"),
            dbc.Col(
                dcc.Dropdown(
                    id='elbow-mode-dropdown',
                    options=[
                        {'label': 'Elbow Up', 'value': 'up'},      # Elbow-up (default, matches paper)
                        {'label': 'Elbow Down', 'value': 'down'}   # Elbow-down (alternative solution)
                    ],
                    value='up',              # Default to elbow up (paper's approach)
                    clearable=False          # Don't allow clearing the selection
                ),
                width=9
            ),
        ], className="mb-3 align-items-center"),
        
        # Solve button (triggers IK calculation)
        dbc.Button(
            "Calculate Inverse Kinematics",  # Button text
            id='solve-button',                # Unique ID for callback
            color="primary",                  # Bootstrap primary color (blue)
            n_clicks=0,                       # Initialize click counter
            className="w-100"                 # Full width button
        ),
    ])
], color="light")  # Light gray background for card

# ------------------------------------------------------------------
# Output Card: Contains angle and position tables
# ------------------------------------------------------------------
outputs_card = dbc.Card([
    dbc.CardBody([
        # Joint angles table title
        html.H6("Joint Angles (deg)", className="card-title"),
        
        # Joint angles DataTable (θ1 through θ5)
        dash_table.DataTable(
            id='joint-angles-table',         # Unique ID for callback to update
            columns=[
                {'name': 'Joint', 'id': 'Joint'},              # Column 1: Joint name
                {'name': 'Angle (deg)', 'id': 'Angle (deg)'}   # Column 2: Angle value
            ],
            style_header=table_header_style,  # Blue header
            style_cell=table_cell_style,      # Monospace font
        ),
        
        # Horizontal rule separator
        html.Hr(),
        
        # Joint positions table title
        html.H6("Cartesian Configuration of Each Joint (mm)", className="card-title"),
        
        # Joint positions DataTable (positions of joints 1-5)
        dash_table.DataTable(
            id='joint-positions-table',      # Unique ID for callback
            columns=[
                {'name': 'Joint', 'id': 'Joint'},       # Column 1: Joint description
                {'name': 'Px (mm)', 'id': 'Px (mm)'},   # Column 2: X coordinate
                {'name': 'Py (mm)', 'id': 'Py (mm)'},   # Column 3: Y coordinate
                {'name': 'Pz (mm)', 'id': 'Pz (mm)'}    # Column 4: Z coordinate
            ],
            style_header=table_header_style,
            style_cell=table_cell_style,
        ),
    ])
], color="light")

# ==============================================================================
# MAIN APPLICATION LAYOUT
# ==============================================================================

# Create the initial "home" figure for the robot arm
initial_home_thetas = [0, 0, 0, 0, 90]
initial_home_positions = kin.get_all_joint_positions(initial_home_thetas)
initial_figure = plot_robot_arm(initial_home_positions)

app.layout = dbc.Container([
    # Store for previous and current joint positions for animation
    dcc.Store(id='joint-positions-store', data={'prev': initial_home_positions.tolist(), 'current': initial_home_positions.tolist()}),
    # Application title (large, centered, with margins)
    html.H1(
        "5-DOF Robotic Arm - Inverse Kinematics Simulator",
        className="text-center my-4"  # Center text, vertical margins
    ),
    
    # Error message alert (hidden by default, shown when IK fails)
    dbc.Alert(
        id='error-message-alert',    # Unique ID for callback to update
        color="danger",               # Red color for errors
        is_open=False,                # Hidden initially
        dismissable=True              # User can close it with X button
    ),
    
    # Main content row with two columns
    dbc.Row([
        # Left column: Input and output cards (60% width)
        dbc.Col([
            inputs_card,                          # Target position inputs
            html.Div(className="mb-3"),           # Spacing between cards
            outputs_card                          # Results tables
        ], width=7),  # 7/12 columns
        
        # Right column: 3D visualization (40% width)
        dbc.Col([
            dcc.Graph(
                id='3d-plot-graph',               # Unique ID for callback
                figure=initial_figure,            # Set the initial figure
                style={'height': '600px'}         # Fixed height for consistency
            )
        ], width=5),  # 5/12 columns
    ], className="mb-4")  # Margin bottom for page padding
], fluid=True)  # Fluid container uses full width of screen

# ==============================================================================
# CALLBACK: MAIN APPLICATION LOGIC
# ==============================================================================

@app.callback(
    # ------------------------------------------------------------------
    # Outputs: What this callback will update
    # ------------------------------------------------------------------
    [
        Output('joint-positions-store', 'data'),
        Output('joint-angles-table', 'data'),        # Update angles table
        Output('joint-positions-table', 'data'),     # Update positions table
        Output('error-message-alert', 'children'),   # Error message text
        Output('error-message-alert', 'is_open')     # Show/hide error alert
    ],
    
    # ------------------------------------------------------------------
    # Inputs: What triggers this callback
    # ------------------------------------------------------------------
    [Input('solve-button', 'n_clicks')],  # Trigger on button click
    
    # ------------------------------------------------------------------
    # States: Values read when callback runs (don't trigger callback)
    # ------------------------------------------------------------------
    [
        State('px-input', 'value'),          # Read Px value
        State('py-input', 'value'),          # Read Py value
        State('pz-input', 'value'),          # Read Pz value
        State('elbow-mode-dropdown', 'value'), # Read elbow mode selection
        State('joint-positions-store', 'data')
    ],
    
    # Don't run callback on page load (only when button clicked)
    prevent_initial_call=True
)
def update_simulation(n_clicks, px, py, pz, elbow_mode, joint_data):
    """
    Main callback function that orchestrates the entire IK calculation and display.
    
    This function is called whenever the "Calculate Inverse Kinematics" button
    is clicked. It performs the following steps:
        1. Validate inputs (ensure all values provided)
        2. Use user-selected elbow configuration (up or down)
        3. Call IK solver from kinematics module
        4. Generate 3D visualization of result
        5. Format output tables
        6. Handle errors gracefully (show home position on failure)
    
    Args:
        n_clicks: Number of times button has been clicked (not used, just a trigger)
        px: X-coordinate of target position (from input field)
        py: Y-coordinate of target position (from input field)
        pz: Z-coordinate of target position (from input field)
        elbow_mode: User's elbow configuration selection ('up' or 'down')
    
    Returns:
        Tuple of 5 elements (matching the 5 Outputs):
            1. Plotly figure object (3D visualization)
            2. List of dicts for angles table
            3. List of dicts for positions table
            4. String for error message (empty if no error)
            5. Boolean for error alert visibility (False if no error)
    
    Error Handling:
        - Missing inputs: Shows error, no update to outputs
        - Unreachable target: Shows home position, displays error message
        - Unexpected errors: Shows error message, no update to outputs
    """
    
    # ------------------------------------------------------------------
    # VALIDATION: Check that all inputs are provided
    # ------------------------------------------------------------------
    if px is None or py is None or pz is None:
        # If any input is missing, don't update plots but show error
        return (
            dash.no_update,  # Don't change 3D plot
            dash.no_update,  # Don't change angles table
            dash.no_update,  # Don't change positions table
            "Error: All position inputs (Px, Py, Pz) must be provided.",  # Error text
            True             # Show error alert
        )
    
    try:
        # ------------------------------------------------------------------
        # STEP 1: Convert inputs to floats (handle string inputs)
        # ------------------------------------------------------------------
        px_val, py_val, pz_val = float(px), float(py), float(pz)

        # ------------------------------------------------------------------
        # STEP 2: Use user-selected elbow configuration (no override)
        # ------------------------------------------------------------------
        # The user's selection is now respected ('up' or 'down')
        resolved_mode = elbow_mode if elbow_mode in ['up', 'down'] else 'up'

        # ------------------------------------------------------------------
        # STEP 3: Call inverse kinematics solver with selected configuration
        # ------------------------------------------------------------------
        # The solve_ik function now properly handles both elbow modes
        ik_res = kin.solve_ik(px_val, py_val, pz_val, elbow_mode=resolved_mode)
        thetas_deg = ik_res["angles"]           # Extract angles list
        joint_positions = np.asarray(ik_res["joints"], dtype=float)  # Extract positions

        # ------------------------------------------------------------------
        # STEP 4: Generate visualization and format tables
        # ------------------------------------------------------------------
        new_joint_positions = np.asarray(ik_res["joints"], dtype=float)
        
        # Get previous positions, if available, otherwise use current as prev
        prev_joint_positions = joint_data.get('current')
        if prev_joint_positions is None:
            prev_joint_positions = new_joint_positions.tolist()

        
        angles_data = create_angles_table(thetas_deg)   # Format angles for table
        positions_data = create_positions_table(joint_positions)  # Format positions
        
        # Return all outputs (no error)
        return {'prev': prev_joint_positions, 'current': new_joint_positions.tolist()}, angles_data, positions_data, "", False

    except ValueError as e:
        # ------------------------------------------------------------------
        # ERROR HANDLING: Target unreachable (math domain error in IK)
        # ------------------------------------------------------------------
        # When IK fails (e.g., target outside workspace), show a "home" position
        # instead of crashing or showing nothing
        
        # Define home position (all joints at 0° except θ5=90°)
        home_thetas = [0, 0, 0, 0, 90]
        
        try:
            # Try to compute FK for home position
            home_positions = kin.get_all_joint_positions(home_thetas)
        except Exception:
            # If even home position fails, use hardcoded fallback
            home_positions = np.array([
                [0, 0, 0],      # Base
                [0, 0, 105],    # Joint 1 (at height d1)
                [105, 0, 105],  # Joint 2 (after extending a2)
                [205, 0, 105],  # Joint 3 (after extending a3)
                [205, 0, 105],  # Joint 4 (same as joint 3, no offset)
                [205, 0, 255],  # Joint 5 (after extending d5)
            ], dtype=float)

        # Create visualization of home position
        fig = plot_robot_arm(home_positions)
        
        # Return home position with error message
        return (
            dash.no_update,                                      # Show home position plot
            create_angles_table(home_thetas),         # Show home angles
            create_positions_table(home_positions),   # Show home positions
            f"Error: Target Unreachable. {e}",        # Display error details
            True                                      # Show error alert
        )

    except Exception as e:
        # ------------------------------------------------------------------
        # ERROR HANDLING: Unexpected errors (programming bugs)
        # ------------------------------------------------------------------
        # Catch-all for any other errors (shouldn't happen in normal use)
        return (
            dash.no_update,                          # Don't change plot
            dash.no_update,                          # Don't change tables
            dash.no_update,
            f"Unexpected error: {e}",                # Show error message
            True                                     # Show error alert
        )

# ==============================================================================
# CLIENTSIDE CALLBACK FOR ANIMATION
# ==============================================================================
clientside_callback(
    """
    function(joint_data, figure) {
        if (!joint_data.prev || !joint_data.current) {
            return figure;
        }

        const duration = 1000;
        const start_time = performance.now();
        const prev = joint_data.prev;
        const current = joint_data.current;

        function get_cylinder_mesh(p1, p2, radius, n_sides = 20) {
            const length = Math.sqrt(Math.pow(p2[0] - p1[0], 2) + Math.pow(p2[1] - p1[1], 2) + Math.pow(p2[2] - p1[2], 2));
            if (length === 0) return {x: [], y: [], z: []};

            const axis = [(p2[0] - p1[0]) / length, (p2[1] - p1[1]) / length, (p2[2] - p1[2]) / length];
            
            let perp_axis;
            if (Math.abs(axis[2]) < 0.99) {
                perp_axis = [axis[1] * 1 - axis[2] * 0, axis[2] * 0 - axis[0] * 1, axis[0] * 0 - axis[1] * 0];
            } else {
                perp_axis = [axis[1] * 0 - axis[2] * 1, axis[2] * 0 - axis[0] * 0, axis[0] * 1 - axis[1] * 0];
            }
            const perp_norm = Math.sqrt(perp_axis[0]*perp_axis[0] + perp_axis[1]*perp_axis[1] + perp_axis[2]*perp_axis[2]);
            perp_axis = [perp_axis[0]/perp_norm, perp_axis[1]/perp_norm, perp_axis[2]/perp_norm];

            const perp_axis2 = [axis[1] * perp_axis[2] - axis[2] * perp_axis[1], axis[2] * perp_axis[0] - axis[0] * perp_axis[2], axis[0] * perp_axis[1] - axis[1] * perp_axis[0]];
            
            const t = Array.from({length: n_sides + 1}, (_, i) => i * 2 * Math.PI / n_sides);
            
            const circ_pts = t.map(angle => [
                radius * (Math.cos(angle) * perp_axis[0] + Math.sin(angle) * perp_axis2[0]),
                radius * (Math.cos(angle) * perp_axis[1] + Math.sin(angle) * perp_axis2[1]),
                radius * (Math.cos(angle) * perp_axis[2] + Math.sin(angle) * perp_axis2[2])
            ]);

            const p_start = circ_pts.map(pt => [p1[0] + pt[0], p1[1] + pt[1], p1[2] + pt[2]]);
            const p_end = circ_pts.map(pt => [p2[0] + pt[0], p2[1] + pt[1], p2[2] + pt[2]]);

            let x = [], y = [], z = [];
            for (let i = 0; i < p_start.length; i++) {
                x.push(p_start[i][0], p_end[i][0]);
                y.push(p_start[i][1], p_end[i][1]);
                z.push(p_start[i][2], p_end[i][2]);
            }
            return {x, y, z};
        }

        function animate() {
            const now = performance.now();
            const time_fraction = Math.min((now - start_time) / duration, 1);

            const interpolated_positions = prev.map((start, i) => {
                const end = current[i];
                return start.map((val, j) => val + (end[j] - val) * time_fraction);
            });
            
            const new_figure = JSON.parse(JSON.stringify(figure));
            const link_radii = [15, 12, 12, 10, 8];

            for (let i = 0; i < interpolated_positions.length - 1; i++) {
                const p1 = interpolated_positions[i];
                const p2 = interpolated_positions[i+1];
                const cyl_mesh = get_cylinder_mesh(p1, p2, link_radii[i]);
                
                new_figure.data[i].x = cyl_mesh.x;
                new_figure.data[i].y = cyl_mesh.y.map(y => -y);
                new_figure.data[i].z = cyl_mesh.z;
            }

            const jp = interpolated_positions;
            const x_plot = jp.map(p => p[0]);
            const y_plot = jp.map(p => -p[1]);
            const z_plot = jp.map(p => p[2]);

            new_figure.data[5].x = x_plot;
            new_figure.data[5].y = y_plot;
            new_figure.data[5].z = z_plot;

            Plotly.react('3d-plot-graph', new_figure);

            if (time_fraction < 1) {
                requestAnimationFrame(animate);
            }
        }
        
        requestAnimationFrame(animate);
        return dash_clientside.no_update;
    }
    """,
    Output('3d-plot-graph', 'figure'),
    Input('joint-positions-store', 'data'),
    State('3d-plot-graph', 'figure'),
    prevent_initial_call=True
)


# ==============================================================================
# INITIAL FIGURE CALLBACK
# ==============================================================================
@app.callback(
    Output('3d-plot-graph', 'figure', allow_duplicate=True),
    Input('solve-button', 'n_clicks'),
    prevent_initial_call='initial_duplicate'
)
def initial_figure_callback(n_clicks):
    if n_clicks and n_clicks > 0:
        return dash.no_update
    # This callback is now only used to prevent updates on initial load,
    # the figure is already created in the layout.
    return dash.no_update


# ==============================================================================
# RUN APPLICATION
# ==============================================================================

# If this file is run directly (not imported), start the Dash server
if __name__ == '__main__':
    # Start the development server
    # debug=True enables:
    #   - Automatic reloading on code changes
    #   - Detailed error messages in browser
    #   - Development tools in browser console
    app.run(debug=True)