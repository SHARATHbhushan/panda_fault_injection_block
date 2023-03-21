import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import plotly.graph_objs as go
from matplotlib.cm import ScalarMappable
import plotly.express as px
import plotly
# Load data from DataFrame
df = pd.read_csv("/home/acefly/robot_fib/data_v3/fault_effect/fault_effect_no_repetation.csv")


  

fig = px.histogram(df, x="fault", color = "time_label")
plotly.offline.plot(fig, filename='/home/acefly/robot_fib/plots/fault_and_time_label.html')


fig2 = px.histogram(df, x="fault_effect", color="time_label")
plotly.offline.plot(fig2, filename='/home/acefly/robot_fib/plots/fault_effect_and_time_label.html')

fig3 = px.density_heatmap(df, x="fault", y="fault_effect",
                         title="Density Map of fault and fault_efffect",
                         marginal_x="histogram",
                         marginal_y="histogram",
                         width=800, height=500)
plotly.offline.plot(fig3, filename='/home/acefly/robot_fib/plots/fault_and_fault_effect.html')

fig4 = px.density_heatmap(df, x="fault", y="fault_effect",
       title="Density Map of fault and fault_effect based on time label", 
       facet_col="time_label")
fig4.show()

fig5 = go.Figure(go.Histogram2d(
        x=df["fault"],
        y=df["fault_effect"],
        texttemplate= "%{z}"
    ))

fig5.show()