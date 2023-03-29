import pandas as pd
import plotly
import plotly.graph_objs as go
import plotly.express as px

#Read cars data from csv
data = pd.read_csv("/home/acefly/robot_fib/data_v4/fault_effect/fault_effect_no_repetation.csv")


fig = px.scatter_3d(data, x='joint', y='pose', z='time_label',
              color='fault_effect',opacity=0.8)

plotly.offline.plot(fig, filename='/home/acefly/robot_fib/plots/Scatter3d_fault_effect_new.html')

'''
#Set marker properties
markercolor = data['fault_effect']

print(markercolor)
#Make Plotly figure
fig1 = go.Scatter3d(x=data['joint'],
                    y=data['pose'],
                    z=data['fault'],
                    marker=dict(color=markercolor,
                                opacity=1,
                                reversescale=True,
                                colorscale='Viridis',
                                size=6),
                    line=dict (width=0.02),
                    mode='markers')

#Make Plot.ly Layout
mylayout = go.Layout(scene=dict(xaxis=dict( title="joint"),
                                yaxis=dict( title="pose"),
                                zaxis=dict(title="fault")),)

#Plot and save html
plotly.offline.plot({"data": [fig1],
                     "layout": mylayout},
                     auto_open=True,
                     filename=("4DPlot.html"))

'''