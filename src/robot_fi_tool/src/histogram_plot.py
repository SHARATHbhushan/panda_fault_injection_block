import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import plotly.graph_objs as go
from matplotlib.cm import ScalarMappable
# Load data from DataFrame
df = pd.read_csv("/home/acefly/robot_fib/data_v3/fault_effect/fault_effect_no_repetation.csv")


faults = df['fault']
fault_effects = df['fault_effect']
# Calculate the frequency of each combination of fault and fault effect
print(len(fault_effects))
print(len(faults))
hist, x_edges, y_edges = np.histogram2d(df['fault'], df['fault_effect'], bins=[len(faults), len(fault_effects)])
hist = hist.T
# Define the size of each bin in the x and y dimensions
dx = x_edges[1] - x_edges[0]
dy = y_edges[1] - y_edges[0]

# Create a meshgrid of the x and y bin centers
x_centers = x_edges[:-1] + dx / 2
y_centers = y_edges[:-1] + dy / 2
x_mesh, y_mesh = np.meshgrid(x_centers, y_centers)

# Create a 3D histogram plot using Plotly
fig = go.Figure(data=[go.Surface(z=hist, x=x_mesh, y=y_mesh, colorscale='Viridis')])

# Set the labels and title
fig.update_layout(scene=dict(xaxis_title='Fault', yaxis_title='Fault Effect', zaxis_title='Frequency'),
                  title='3D Histogram of Fault and Fault Effect')

# Plot the figure
fig.show()