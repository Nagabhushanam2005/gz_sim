import matplotlib.pyplot as plt
import numpy as np
import plotly.graph_objects as go

def read_data(filename):
    with open(filename) as f:
        return np.loadtxt(f, comments='#')

# Plot wave height
odom_data = read_data('odom.dat')
plt.figure(figsize=(10, 5))
plt.plot(odom_data[:,0], odom_data[:,1])
plt.xlabel('Simulation Time (s)')
plt.ylabel('Wave Height (m)')
plt.title('Wave Height vs Time')
plt.grid(True)
plt.savefig('odom.png', dpi=300, bbox_inches='tight')

# Plot error angle
angles_data = read_data('angles.dat')
error_data = np.abs(angles_data[:,1] - 90)
plt.figure(figsize=(10, 5))
plt.plot(angles_data[:,0], angles_data[:,1])
plt.plot(angles_data[:,0], error_data, label='Error Angle')
plt.xlabel('Simulation Time (s)')
plt.ylabel('Error Angle (deg)')
plt.title('Wave Impact Angle Deviation (from 90°)')
plt.grid(True)
plt.savefig('angles.png', dpi=300, bbox_inches='tight')

# Plot force components
force_data = read_data('force.dat')
plt.figure(figsize=(10, 5))
plt.plot(force_data[:,0], force_data[:,1], label='Fx')
plt.plot(force_data[:,0], force_data[:,2], label='Fy')
plt.plot(force_data[:,0], force_data[:,3], label='Fz')
plt.xlabel('Simulation Time (s)')
plt.ylabel('Force (N)')
plt.title('Hydrodynamic Force Components')
plt.legend()
plt.grid(True)
plt.savefig('force.png', dpi=300, bbox_inches='tight')

pose_data = read_data('pose.dat')   # <sim_time> <x> <y> <z> <qx> <qy> <qz> <qw>
force_data = read_data('force.dat') # <sim_time> <Fx> <Fy> <Fz>

# Extract positions and forces
x, y, z = pose_data[:,1], pose_data[:,2], pose_data[:,3]
fx, fy, fz = force_data[:,1], force_data[:,2], force_data[:,3]

# Compute force magnitude for tooltips
force_mags = np.linalg.norm(np.stack([fx, fy, fz], axis=1), axis=1)

hover_text = [
    f"Force: {mag:.2f} N<br>Fx: {fxi:.2f} N<br>Fy: {fyi:.2f} N<br>Fz: {fzi:.2f} N"
    for mag, fxi, fyi, fzi in zip(force_mags, fx, fy, fz)
]

# Create interactive trajectory with hover tooltips
trajectory = go.Scatter3d(
    x=x, y=y, z=z,
    mode='lines+markers',
    marker=dict(
        size=3,
        color=force_mags,
        colorscale='Viridis',
        colorbar=dict(title='Force (N)')
    ),
    line=dict(color='grey', width=2),
    name='Trajectory',
    text=hover_text,
    hoverinfo='text'
)

indices = np.linspace(0, len(x) - 1, 10, dtype=int)

fig = go.Figure(data=[trajectory])
fig.update_layout(
    scene=dict(
        xaxis_title='X (m)',
        yaxis_title='Y (m)',
        zaxis_title='Z (m)'
    ),
    title='Trajectory with Sampled Force Vectors (Hover for Details)',
    margin=dict(l=0, r=0, b=0, t=30)
)

fig.write_html("trajectory_with_forces.html")
fig.show()

