"""
app.py
Creates an interactive web application (Plotly Dash) to control and 
visualize the 5-DOF robotic arm.

This application replicates the functionality of the MATLAB GUI shown 
in the paper, providing:
    - User input fields for target position (Px, Py, Pz)
    - Elbow configuration selector (forced to elbow-up for paper consistency)
    - 3D visualization of robot arm using Plotly
    - Output tables showing joint angles and positions
    - Error handling with fallback to home position
"""

# Core Dash framework imports
import dash
from dash import dcc, html, dash_table, Input, Output, State

# Plotly for 3D visualization
import plotly.graph_objects as go

# Bootstrap components for professional UI layout
import dash_bootstrap_components as dbc

# NumPy for array operations
import numpy as np

# Import our custom kinematics module (IK/FK solvers)
import kinematics as kin

# ==============================================================================
# CONFIGURATION AND HELPER FUNCTIONS
# ==============================================================================

def force_elbow_up(_: float, __: str) -> str:
    """
    Force elbow-up configuration regardless of user selection or target position.
    
    This function overrides any elbow mode selection to ensure consistency with
    the research paper, which exclusively uses elbow-up configuration (θ3 < 0)
    in both validation cases.
    
    Args:
        _: Target Px value (unused, kept for function signature compatibility)
        __: User's elbow mode selection (unused, will be overridden)
    
    Returns:
        Always returns "up" to force elbow-up configuration
    
    Rationale:
        The paper's screenshots and angle tables both show elbow-up poses.
        While the mathematics supports elbow-down, we prioritize paper-faithful
        reproduction for validation purposes.
    """
    return "up"

# ==============================================================================
# VISUALIZATION FUNCTIONS
# ==============================================================================

def plot_robot_arm(joint_positions):
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
    # Convert input to numpy array and ensure float type
    jp = np.asarray(joint_positions, dtype=float)

    # ------------------------------------------------------------------
    # Axis transformation for visualization alignment
    # ------------------------------------------------------------------
    # The paper's screenshots show a specific view angle. To match it:
    # - Keep X data as-is, but reverse axis direction via range parameter
    # - Negate Y data AND reverse axis to create right-to-left orientation
    # - Keep Z standard (vertical, upward positive)
    
    x_plot = jp[:, 0]       # X coordinates (no data transformation)
    y_plot = -jp[:, 1]      # Y coordinates (negated for visualization)
    z_plot = jp[:, 2]       # Z coordinates (no transformation)

    # Initialize empty Plotly figure
    fig = go.Figure()

    # ------------------------------------------------------------------
    # Add arm links (lines connecting consecutive joints)
    # ------------------------------------------------------------------
    fig.add_trace(go.Scatter3d(
        x=x_plot,                      # X coordinates of all 6 points
        y=y_plot,                      # Y coordinates (transformed)
        z=z_plot,                      # Z coordinates
        mode='lines',                  # Draw lines between points
        line=dict(width=10),           # Thick line for visibility
        name='Arm Links'               # Legend label
    ))
    
    # ------------------------------------------------------------------
    # Add joint markers (circles at each joint position)
    # ------------------------------------------------------------------
    fig.add_trace(go.Scatter3d(
        x=x_plot,                      # Same coordinates as links
        y=y_plot,
        z=z_plot,
        mode='markers',                # Draw only markers, no lines
        marker=dict(size=8, symbol='circle'),  # Medium-sized circles
        name='Joints'                  # Legend label
    ))

    # ------------------------------------------------------------------
    # Add red baseline from base to z = -400 (visual reference)
    # ------------------------------------------------------------------
    # This vertical line helps visualize the base height and provides
    # a reference for understanding the robot's vertical extent
    fig.add_trace(go.Scatter3d(
        x=[0, 0],                      # Vertical line at x=0
        y=[0, 0],                      # And y=0 (at origin in XY plane)
        z=[0, -400],                   # From base down to bottom of view
        mode='lines',                  # Draw as line
        line=dict(color='red', width=6),  # Red color, thick for visibility
        name='Base Line',              # Legend label
        showlegend=False               # Hide from legend (it's just a reference)
    ))

    # ------------------------------------------------------------------
    # Define symmetric axis ranges for undistorted visualization
    # ------------------------------------------------------------------
    # Using symmetric ranges ensures the robot appears with correct proportions
    AX_MIN, AX_MAX = -400.0, 400.0

    # ------------------------------------------------------------------
    # Configure plot layout and 3D scene properties
    # ------------------------------------------------------------------
    fig.update_layout(
        title="3D Simulation",         # Plot title
        scene=dict(
            # X-axis: Left to right (achieved by reversing range)
            xaxis=dict(
                title='X (mm)',        # Axis label with units
                range=[AX_MAX, AX_MIN],  # REVERSED: [400, -400] makes right=negative
                autorange=False        # Disable autorange to maintain fixed view
            ),
            
            # Y-axis: Right to left (achieved by reversing range + data negation)
            yaxis=dict(
                title='Y (mm)',
                range=[AX_MAX, AX_MIN],  # REVERSED: [400, -400]
                autorange=False
            ),
            
            # Z-axis: Standard vertical (bottom to top)
            zaxis=dict(
                title='Z (mm)',
                range=[AX_MIN, AX_MAX],  # Normal: [-400, 400]
                autorange=False
            ),
            
            # Maintain cubic aspect ratio (prevents distortion)
            aspectmode='cube',
            
            # Camera configuration for optimal viewing angle
            camera=dict(
                up=dict(x=0, y=0, z=1),        # Z-axis points up
                center=dict(x=0, y=0, z=0),    # Look at origin
                eye=dict(x=2.2, y=0.8, z=1.4), # Camera position (tuned for clarity)
            ),
        ),
        
        # Minimize margins to maximize plot area
        margin=dict(l=0, r=0, b=0, t=40),
        
        # Hide legend (we don't need it for this simple visualization)
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
        
        # Elbow configuration dropdown (currently overridden to "up")
        dbc.Row([
            dbc.Col(dbc.Label("Elbow Config:"), width=3, className="text-end"),
            dbc.Col(
                dcc.Dropdown(
                    id='elbow-mode-dropdown',
                    options=[
                        {'label': 'Auto (Paper-based)', 'value': 'auto'},  # Automatic selection
                        {'label': 'Elbow Up', 'value': 'up'},              # Force elbow up
                        {'label': 'Elbow Down', 'value': 'down'}           # Force elbow down
                    ],
                    value='up',              # Default to elbow up
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

app.layout = dbc.Container([
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
        Output('3d-plot-graph', 'figure'),           # Update 3D visualization
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
        State('elbow-mode-dropdown', 'value') # Read elbow mode selection
    ],
    
    # Don't run callback on page load (only when button clicked)
    prevent_initial_call=True
)
def update_simulation(n_clicks, px, py, pz, elbow_mode):
    """
    Main callback function that orchestrates the entire IK calculation and display.
    
    This function is called whenever the "Calculate Inverse Kinematics" button
    is clicked. It performs the following steps:
        1. Validate inputs (ensure all values provided)
        2. Force elbow-up configuration (paper consistency)
        3. Call IK solver from kinematics module
        4. Generate 3D visualization of result
        5. Format output tables
        6. Handle errors gracefully (show home position on failure)
    
    Args:
        n_clicks: Number of times button has been clicked (not used, just a trigger)
        px: X-coordinate of target position (from input field)
        py: Y-coordinate of target position (from input field)
        pz: Z-coordinate of target position (from input field)
        elbow_mode: User's elbow configuration selection (will be overridden)
    
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
        # STEP 2: Force elbow-up configuration for paper consistency
        # ------------------------------------------------------------------
        # This overrides any user selection to ensure we match paper results
        resolved_mode = force_elbow_up(px_val, elbow_mode)

        # ------------------------------------------------------------------
        # STEP 3: Call inverse kinematics solver
        # ------------------------------------------------------------------
        # Check which IK function is available (handles different versions)
        if hasattr(kin, "solve_ik"):
            # Newer version: solve_ik returns dict with angles, joints, meta
            ik_res = kin.solve_ik(px_val, py_val, pz_val, elbow_mode=resolved_mode)
            thetas_deg = ik_res["angles"]           # Extract angles list
            joint_positions = np.asarray(ik_res["joints"], dtype=float)  # Extract positions
        else:
            # Older version: calculate_ik returns tuple (angles, intermediates)
            thetas_deg, _ = kin.calculate_ik(px_val, py_val, pz_val, elbow_mode=resolved_mode)
            # Need to call FK separately to get positions
            joint_positions = kin.get_all_joint_positions(thetas_deg)

        # ------------------------------------------------------------------
        # STEP 4: Generate visualization and format tables
        # ------------------------------------------------------------------
        fig = plot_robot_arm(joint_positions)           # Create 3D plot
        angles_data = create_angles_table(thetas_deg)   # Format angles for table
        positions_data = create_positions_table(joint_positions)  # Format positions
        
        # Return all outputs (no error)
        return fig, angles_data, positions_data, "", False

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
            if hasattr(kin, "forward_kinematics"):
                pts, _ = kin.forward_kinematics(home_thetas)
                home_positions = np.asarray(pts, dtype=float)
            else:
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
            fig,                                      # Show home position plot
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
