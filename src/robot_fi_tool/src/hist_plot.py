import plotly.express as px
import pandas as pd
df = pd.read_csv("/home/acefly/robot_fib/data_v3/fault_effect/fault_effect_no_repetation.csv")

fig = px.histogram(df, x="pose")
fig.show()

