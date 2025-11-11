"""
app.py
Creates an interactive web application (Plotly Dash) to control and 
visualize the 5-DOF robotic arm.

This application replicates the functionality of the MATLAB GUI shown 
in the paper.
"""

import dash
from dash import dcc, html, dash_table, Input, Output, State
import plotly.graph_objects as go
import dash_bootstrap_components as dbc
import numpy as np

# Import our custom kinematics functions
import kinematics as kin

# --- 1. PLOTTING HELPER FUNCTION ---

def plot_robot_arm(joint_positions):
    """
    Creates a 3D Plotly figure of the robotic arm.
    
    Args:
        joint_positions (numpy.ndarray): A 6x3 array of [x, y, z] coordinates
                                         (from Frame 0 to Frame 5).
                                         
    Returns:
        plotly.graph_objects.Figure: The 3D scatter/line plot.
    """
    
    # Extract coordinates
    x = joint_positions[:, 0]
    y = joint_positions[:, 1]
    z = joint_positions[:, 2]
    
    fig = go.Figure()
    
    # Add the arm links as a line
    fig.add_trace(go.Scatter3d(
        x=x, y=y, z=z,
        mode='lines',
        line=dict(color='blue', width=10),
        name='Arm Links'
    ))
    
    # Add the joints as markers
    fig.add_trace(go.Scatter3d(
        x=x, y=y, z=z,
        mode='markers',
        marker=dict(color='red', size=8, symbol='circle'),
        name='Joints'
    ))
    
    # Configure the 3D plot layout
    # Set axis limits to be equal for a cubic plot
    max_val = np.max(np.abs(joint_positions)) + 50 # Add 50mm padding
    
    fig.update_layout(
        title="3D Simulation",
        scene=dict(
            xaxis=dict(title='X (mm)', range=[-max_val, max_val]),
            yaxis=dict(title='Y (mm)', range=[-max_val, max_val]),
            zaxis=dict(title='Z (mm)', range=[0, max_val]), # Base is at z=0
            aspectmode='cube' # Ensures a 1:1:1 aspect ratio
        ),
        margin=dict(l=0, r=0, b=0, t=40),
        showlegend=False
    )
    
    return fig

# --- 2. FORMATTING HELPER FUNCTIONS ---

def create_angles_table(thetas_deg):
    """Formats the joint angles for the Dash DataTable."""
    data = [
        {'Joint': 'Theta 1', 'Angle (deg)': f"{thetas_deg[0]:.4f}"},
        {'Joint': 'Theta 2', 'Angle (deg)': f"{thetas_deg[1]:.4f}"},
        {'Joint': 'Theta 3', 'Angle (deg)': f"{thetas_deg[2]:.4f}"},
        {'Joint': 'Theta 4', 'Angle (deg)': f"{thetas_deg[3]:.4f}"},
        {'Joint': 'Theta 5', 'Angle (deg)': f"{thetas_deg[4]:.4f}"},
    ]
    return data

def create_positions_table(joint_positions):
    """Formats the joint positions for the Dash DataTable."""
    data = []
    # Get positions for Joints 1-5 (skip base frame 0)
    for i in range(1, 6):
        pos = joint_positions[i]
        data.append({
            'Joint': f'Position at Joint {i}',
            'Px (mm)': f"{pos[0]:.4f}",
            'Py (mm)': f"{pos[1]:.4f}",
            'Pz (mm)': f"{pos[2]:.4f}",
        })
    return data

# --- 3. INITIALIZE THE DASH APP ---
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.LUX])
server = app.server
app.title = "5-DOF Robot IK"

# --- 4. DEFINE THE APP LAYOUT  ---

# Define table styles
table_header_style = {"backgroundColor": "#007BFF", "color": "white", "fontWeight": "bold"}
table_cell_style = {"textAlign": "left", "fontFamily": "Consolas, monospace"}

# Define components for the "Inputs" card
inputs_card = dbc.Card([
    dbc.CardBody([
        html.H5("Target End-Effector Position", className="card-title"),
        dbc.Row([
            dbc.Col(dbc.Label("Px:"), width=3, className="text-end"),
            dbc.Col(dcc.Input(id='px-input', type='number', value=220.0, step=0.1, className="w-100"), width=9),
        ], className="mb-2 align-items-center"),
        dbc.Row([
            dbc.Col(dbc.Label("Py:"), width=3, className="text-end"),
            dbc.Col(dcc.Input(id='py-input', type='number', value=161.0, step=0.1, className="w-100"), width=9),
        ], className="mb-2 align-items-center"),
        dbc.Row([
            dbc.Col(dbc.Label("Pz:"), width=3, className="text-end"),
            dbc.Col(dcc.Input(id='pz-input', type='number', value=220.0, step=0.1, className="w-100"), width=9),
        ], className="mb-2 align-items-center"),
        html.Hr(),
        dbc.Row([
            dbc.Col(dbc.Label("Elbow Config:"), width=3, className="text-end"),
            dbc.Col(
                dcc.Dropdown(
                    id='elbow-mode-dropdown',
                    options=[
                        {'label': 'Auto (Paper-based)', 'value': 'auto'},
                        {'label': 'Elbow Up', 'value': 'up'},
                        {'label': 'Elbow Down', 'value': 'down'}
                    ],
                    value='auto',
                    clearable=False
                ),
                width=9
            ),
        ], className="mb-3 align-items-center"),
        dbc.Button("Calculate Inverse Kinematics", id='solve-button', color="primary", n_clicks=0, className="w-100"),
    ])
], color="light")

# Define components for the "Outputs" card
outputs_card = dbc.Card([
    dbc.CardBody([
        html.H6("Joint Angles (deg)", className="card-title"),
        dash_table.DataTable(
            id='joint-angles-table',
            columns=[
                {'name': 'Joint', 'id': 'Joint'},
                {'name': 'Angle (deg)', 'id': 'Angle (deg)'},
            ],
            style_header=table_header_style,
            style_cell=table_cell_style,
        ),
        html.Hr(),
        html.H6("Cartesian Configuration of Each Joint (mm)", className="card-title"),
        dash_table.DataTable(
            id='joint-positions-table',
            columns=[
                {'name': 'Joint', 'id': 'Joint'},
                {'name': 'Px (mm)', 'id': 'Px (mm)'},
                {'name': 'Py (mm)', 'id': 'Py (mm)'},
                {'name': 'Pz (mm)', 'id': 'Pz (mm)'},
            ],
            style_header=table_header_style,
            style_cell=table_cell_style,
        ),
    ])
], color="light")

# Define the overall app layout
app.layout = dbc.Container([
    html.H1("5-DOF Robotic Arm - Inverse Kinematics Simulator", className="text-center my-4"),
    dbc.Alert(id='error-message-alert', color="danger", is_open=False, dismissable=True),
    dbc.Row([
        dbc.Col([
            inputs_card,
            html.Div(className="mb-3"),
            outputs_card
        ], width=7),
        dbc.Col([
            dcc.Graph(id='3d-plot-graph', style={'height': '600px'})
        ], width=5),
    ], className="mb-4")
], fluid=True)

# --- 5. DEFINE THE CALLBACK (The App's "Brain") ---
@app.callback(
    [Output('3d-plot-graph', 'figure'),
     Output('joint-angles-table', 'data'),
     Output('joint-positions-table', 'data'),
     Output('error-message-alert', 'children'),
     Output('error-message-alert', 'is_open')],
    [Input('solve-button', 'n_clicks')], # Trigger
    [State('px-input', 'value'),
     State('py-input', 'value'),
     State('pz-input', 'value'),
     State('elbow-mode-dropdown', 'value')],
    prevent_initial_call=True # Do not run on page load
)
def update_simulation(n_clicks, px, py, pz, elbow_mode):
    """
    This is the core function of the web app. It links all
    inputs to all outputs, exactly as the MATLAB GUI does.
    """
    
    if px is None or py is None or pz is None:
        return (dash.no_update, dash.no_update, dash.no_update,
                "Error: All position inputs (Px, Py, Pz) must be provided.", True)
    
    try:
        # --- Step 1: Calculate Inverse Kinematics ---
        thetas_deg, intermediates = kin.calculate_ik(float(px), float(py), float(pz), elbow_mode=elbow_mode)
        
        # --- Step 2: Calculate Forward Kinematics (for Verification) ---
        joint_positions = kin.get_all_joint_positions(thetas_deg)
        
        # --- Step 3: Generate Plotly Figure ---
        fig = plot_robot_arm(joint_positions)
        
        # --- Step 4: Format Output Tables ---
        angles_data = create_angles_table(thetas_deg)
        positions_data = create_positions_table(joint_positions)
        
        # --- Step 5: Return all outputs ---
        # Add debug info about which configuration was used
        config_msg = f"Using configuration: {intermediates.get('elbow_config', 'unknown')}"
        return fig, angles_data, positions_data, "", False # No error
        
    except ValueError as e:
        # Catch "Target unreachable" or other math errors
        home_thetas = [0, 0, 0, 0, 90] # A simple "home" position
        home_positions = kin.get_all_joint_positions(home_thetas)
        fig = plot_robot_arm(home_positions)
        
        return (
            fig, # Show home position
            create_angles_table(home_thetas),
            create_positions_table(home_positions),
            f"Error: Target Unreachable. {e}", # Display the error message
            True # Open the error alert
        )

# --- 6. RUN THE APPLICATION ---
if __name__ == '__main__':
    app.run(debug=True)