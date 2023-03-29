import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import plotly.graph_objs as go
from matplotlib.cm import ScalarMappable
import plotly.express as px
import plotly
# Load data from DataFrame
df = pd.read_csv("/home/acefly/robot_fib/data_v4/fault_effect/fault_effect_no_repetation.csv")


x_test_size = 35

df["fault"] = df["fault"].replace(to_replace=1,value="Noise")
df["fault"] = df["fault"].replace(to_replace=2,value="Stuck_at")
df["fault"] = df["fault"].replace(to_replace=3,value="package_drop")
df["fault"] = df["fault"].replace(to_replace=4,value="offset")


df["fault_effect"] = df["fault_effect"].replace(to_replace=1,value="Tool Thrown")
df["fault_effect"] = df["fault_effect"].replace(to_replace=2,value="Human Collision")
df["fault_effect"] = df["fault_effect"].replace(to_replace=3,value="back cell collision")
df["fault_effect"] = df["fault_effect"].replace(to_replace=4,value="Right cell collision")
df["fault_effect"] = df["fault_effect"].replace(to_replace=5,value="Front cell collision")
df["fault_effect"] = df["fault_effect"].replace(to_replace=6,value="Controller failure")


df["time_label"] = df["time_label"].replace(to_replace=2,value="Planning")
df["time_label"] = df["time_label"].replace(to_replace=3,value="Execution")



fig = px.histogram(df, x="fault", color = "time_label",opacity=0.7)
fig.update_xaxes(
        tickangle = 0,
        title_text = "Fault",
        title_font = {"size": 35},
        title_standoff = 25)

fig.update_yaxes(
        tickangle = 0,
        title_text = "count",
        title_font = {"size": 35},
        title_standoff = 25)
fig.update_layout(xaxis_tickfont=dict(size=20), yaxis_tickfont=dict(size=20))

plotly.offline.plot(fig, filename='/home/acefly/robot_fib/plot_v2/fault_and_time_label.html')





fig2 = px.histogram(df, x="fault_effect", color="time_label",opacity=0.7)

fig2.update_xaxes(
        tickangle = 0,
        title_text = "Fault Effect",
        title_font = {"size": 35},
        title_standoff = 25)

fig2.update_yaxes(
        tickangle = 0,
        title_text = "count",
        title_font = {"size": 35},
        title_standoff = 25)

fig2.update_layout(xaxis_tickfont=dict(size=20), yaxis_tickfont=dict(size=20))


plotly.offline.plot(fig2, filename='/home/acefly/robot_fib/plot_v2/fault_effect_and_time_label.html')

fig3 = px.density_heatmap(df, x="fault", y="fault_effect",
                         marginal_x="histogram",
                         marginal_y="histogram")


fig3.update_xaxes(
        tickangle = 0,
        title_font = {"size": 35},
        title_standoff = 25)

fig3.update_yaxes(
        tickangle = 0,
        title_font = {"size": 35},
        title_standoff = 25)

fig3.update_layout(coloraxis = {'colorscale':'viridis'})
fig3.update_layout(xaxis_tickfont=dict(size=20), yaxis_tickfont=dict(size=20))
plotly.offline.plot(fig3, filename='/home/acefly/robot_fib/plot_v2/fault_and_fault_effect.html')





fig4 = px.density_heatmap(df, x="fault", y="fault_effect",
       facet_col="time_label",

       )

fig4.update_layout(coloraxis = {'colorscale':'viridis'})





plotly.offline.plot(fig4, filename='/home/acefly/robot_fib/plot_v2/fault_and_fault_effect_based_on_time_label.html')




fig5 = go.Figure(go.Histogram2d(
        x=df["fault"],
        y=df["fault_effect"],
        texttemplate= "%{z}",
        textfont=dict(size=30),
        colorscale = 'viridis'
    ))


fig5.update_xaxes(
        tickangle = 0,
        title_text = "Fault",
        title_font = {"size": 35},
        title_standoff = 25)

fig5.update_yaxes(
        tickangle = 0,
        title_text = "Fault Effect",
        title_font = {"size": 35},
        title_standoff = 25)
fig5.update_layout(coloraxis = {'colorscale':'Bluered'})
fig5.update_layout(xaxis_tickfont=dict(size=20), yaxis_tickfont=dict(size=20))

plotly.offline.plot(fig5, filename='/home/acefly/robot_fib/plot_v2/fault_and_fault_effect_histogram2d.html')
