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

# --- 0) Force elbow-up (paper shows elbow-up in both cases) ---
def force_elbow_up(_: float, __: str) -> str:
    return "up"

# --- 1. PLOTTING (axes orientation to match the paper) ---
def plot_robot_arm(joint_positions):
    """
    Creates a 3D Plotly figure of the robotic arm.

    Args:
        joint_positions (numpy.ndarray): A 6x3 array of [x, y, z] coordinates
                                         (from Frame 0 to Frame 5).

    Returns:
        plotly.graph_objects.Figure: The 3D scatter/line plot.
    """
    jp = np.asarray(joint_positions, dtype=float)

    # Axes requirements:
    #  - X: left -> right  (flip axis direction in the plot)
    #  - Y: right -> left  (as you confirmed; keep it)
    #  - Z: [-400, 400]
    x_plot = jp[:, 0]       # no data flip; we flip axis orientation via range
    y_plot = -jp[:, 1]      # flip ONLY for visualization to read right->left
    z_plot = jp[:, 2]       # unchanged

    fig = go.Figure()

    # Arm links
    fig.add_trace(go.Scatter3d(
        x=x_plot, y=y_plot, z=z_plot,
        mode='lines',
        line=dict(width=10),
        name='Arm Links'
    ))
    # Joints
    fig.add_trace(go.Scatter3d(
        x=x_plot, y=y_plot, z=z_plot,
        mode='markers',
        marker=dict(size=8, symbol='circle'),
        name='Joints'
    ))

    # Fixed symmetric ranges to match paper
    AX_MIN, AX_MAX = -400.0, 400.0

    fig.update_layout(
        title="3D Simulation",
        scene=dict(
            # X increases visually from left -> right by REVERSING the axis direction:
            # setting range [AX_MAX, AX_MIN] flips the rendered axis.
            xaxis=dict(title='X (mm)', range=[AX_MAX, AX_MIN], autorange=False),
            # Y increases right -> left (reverse the axis direction)
            yaxis=dict(title='Y (mm)', range=[AX_MAX, AX_MIN], autorange=False),
            # Z spans -400 .. 400
            zaxis=dict(title='Z (mm)', range=[AX_MIN, AX_MAX], autorange=False),
            aspectmode='cube',
            camera=dict(
                up=dict(x=0, y=0, z=1),        # Z up
                center=dict(x=0, y=0, z=0),
                eye=dict(x=2.2, y=0.8, z=1.4),
            ),
        ),
        margin=dict(l=0, r=0, b=0, t=40),
        showlegend=False
    )
    return fig

# --- 2. TABLE FORMATTING ---
def create_angles_table(thetas_deg):
    return [
        {'Joint': 'Theta 1', 'Angle (deg)': f"{thetas_deg[0]:.4f}"},
        {'Joint': 'Theta 2', 'Angle (deg)': f"{thetas_deg[1]:.4f}"},
        {'Joint': 'Theta 3', 'Angle (deg)': f"{thetas_deg[2]:.4f}"},
        {'Joint': 'Theta 4', 'Angle (deg)': f"{thetas_deg[3]:.4f}"},
        {'Joint': 'Theta 5', 'Angle (deg)': f"{thetas_deg[4]:.4f}"},
    ]

def create_positions_table(joint_positions):
    # Keep raw numbers (no Y flip) so the table reflects real model coordinates
    jp = np.asarray(joint_positions, dtype=float)
    rows = []
    for i in range(1, 6):  # joints 1..5
        x, y, z = jp[i]
        rows.append({
            'Joint': f'Position at Joint {i}',
            'Px (mm)': f"{x:.4f}",
            'Py (mm)': f"{y:.4f}",
            'Pz (mm)': f"{z:.4f}",
        })
    return rows

# --- 3. DASH APP ---
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.LUX])
server = app.server
app.title = "5-DOF Robot IK"

# --- 4. LAYOUT ---
table_header_style = {"backgroundColor": "#007BFF", "color": "white", "fontWeight": "bold"}
table_cell_style = {"textAlign": "left", "fontFamily": "Consolas, monospace"}

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
                    value='up',
                    clearable=False
                ),
                width=9
            ),
        ], className="mb-3 align-items-center"),
        dbc.Button("Calculate Inverse Kinematics", id='solve-button', color="primary", n_clicks=0, className="w-100"),
    ])
], color="light")

outputs_card = dbc.Card([
    dbc.CardBody([
        html.H6("Joint Angles (deg)", className="card-title"),
        dash_table.DataTable(
            id='joint-angles-table',
            columns=[{'name': 'Joint', 'id': 'Joint'},
                     {'name': 'Angle (deg)', 'id': 'Angle (deg)'}],
            style_header=table_header_style,
            style_cell=table_cell_style,
        ),
        html.Hr(),
        html.H6("Cartesian Configuration of Each Joint (mm)", className="card-title"),
        dash_table.DataTable(
            id='joint-positions-table',
            columns=[{'name': 'Joint', 'id': 'Joint'},
                     {'name': 'Px (mm)', 'id': 'Px (mm)'},
                     {'name': 'Py (mm)', 'id': 'Py (mm)'},
                     {'name': 'Pz (mm)', 'id': 'Pz (mm)'}],
            style_header=table_header_style,
            style_cell=table_cell_style,
        ),
    ])
], color="light")

app.layout = dbc.Container([
    html.H1("5-DOF Robotic Arm - Inverse Kinematics Simulator", className="text-center my-4"),
    dbc.Alert(id='error-message-alert', color="danger", is_open=False, dismissable=True),
    dbc.Row([
        dbc.Col([inputs_card, html.Div(className="mb-3"), outputs_card], width=7),
        dbc.Col([dcc.Graph(id='3d-plot-graph', style={'height': '600px'})], width=5),
    ], className="mb-4")
], fluid=True)

# --- 5. CALLBACK ---
@app.callback(
    [Output('3d-plot-graph', 'figure'),
     Output('joint-angles-table', 'data'),
     Output('joint-positions-table', 'data'),
     Output('error-message-alert', 'children'),
     Output('error-message-alert', 'is_open')],
    [Input('solve-button', 'n_clicks')],
    [State('px-input', 'value'),
     State('py-input', 'value'),
     State('pz-input', 'value'),
     State('elbow-mode-dropdown', 'value')],
    prevent_initial_call=True
)
def update_simulation(n_clicks, px, py, pz, elbow_mode):
    if px is None or py is None or pz is None:
        return (dash.no_update, dash.no_update, dash.no_update,
                "Error: All position inputs (Px, Py, Pz) must be provided.", True)
    try:
        px_val, py_val, pz_val = float(px), float(py), float(pz)

        # Force elbow-up branch to match the paper's screenshots
        resolved_mode = force_elbow_up(px_val, elbow_mode)

        # Use IK (solve_ik returns angles & joints if available)
        if hasattr(kin, "solve_ik"):
            ik_res = kin.solve_ik(px_val, py_val, pz_val, elbow_mode=resolved_mode)
            thetas_deg = ik_res["angles"]
            joint_positions = np.asarray(ik_res["joints"], dtype=float)
        else:
            thetas_deg, _ = kin.calculate_ik(px_val, py_val, pz_val, elbow_mode=resolved_mode)
            joint_positions = kin.get_all_joint_positions(thetas_deg)

        fig = plot_robot_arm(joint_positions)
        angles_data = create_angles_table(thetas_deg)
        positions_data = create_positions_table(joint_positions)
        return fig, angles_data, positions_data, "", False

    except ValueError as e:
        # Show a simple home pose on errors
        home_thetas = [0, 0, 0, 0, 90]
        try:
            if hasattr(kin, "forward_kinematics"):
                pts, _ = kin.forward_kinematics(home_thetas)
                home_positions = np.asarray(pts, dtype=float)
            else:
                home_positions = kin.get_all_joint_positions(home_thetas)
        except Exception:
            home_positions = np.array([
                [0, 0, 0],
                [0, 0, 105],
                [105, 0, 105],
                [205, 0, 105],
                [205, 0, 105],
                [205, 0, 255],
            ], dtype=float)

        fig = plot_robot_arm(home_positions)
        return (fig,
                create_angles_table(home_thetas),
                create_positions_table(home_positions),
                f"Error: Target Unreachable. {e}",
                True)

    except Exception as e:
        return (dash.no_update, dash.no_update, dash.no_update,
                f"Unexpected error: {e}", True)

# --- 6. RUN ---
if __name__ == '__main__':
    app.run(debug=True)
