import pandas as pd
import pandas as pd
import plotly
import plotly.graph_objs as go
import plotly.express as px
import plotly.data as pdata
import matplotlib.pyplot as plt


df = pd.read_csv("/home/acefly/robot_fib/data_v3/fault_effect/fault_effect_no_repetation.csv")

df = df[['fault', 'time_label', 'fault_effect']]


df_fe1 = df.loc[df["fault_effect"] == 1 ]
df_fe2 = df.loc[df["fault_effect"] == 2 ]
df_fe3 = df.loc[df["fault_effect"] == 3 ]
df_fe4 = df.loc[df["fault_effect"] == 4 ]
df_fe5 = df.loc[df["fault_effect"] == 5 ]
df_fe6 = df.loc[df["fault_effect"] == 6 ]



df_planning_fe1 = df_fe1.loc[df_fe1["time_label"] == 2 ]
df_planning_fe2 = df_fe2.loc[df_fe2["time_label"] == 2 ]
df_planning_fe3 = df_fe3.loc[df_fe3["time_label"] == 2 ]
df_planning_fe4 = df_fe4.loc[df_fe4["time_label"] == 2 ]
df_planning_fe5 = df_fe5.loc[df_fe5["time_label"] == 2 ]
df_planning_fe6 = df_fe6.loc[df_fe6["time_label"] == 2 ]


df_execution_fe1 = df_fe1.loc[df_fe1["time_label"] == 3]
df_execution_fe2 = df_fe2.loc[df_fe2["time_label"] == 3]
df_execution_fe3 = df_fe3.loc[df_fe3["time_label"] == 3]
df_execution_fe4 = df_fe4.loc[df_fe4["time_label"] == 3]
df_execution_fe5 = df_fe5.loc[df_fe5["time_label"] == 3]
df_execution_fe6 = df_fe6.loc[df_fe6["time_label"] == 3]



print(df_planning_fe1.value_counts())

column = df_execution_fe1.columns

df_planning_fe1_val_counts = df_planning_fe1.value_counts().reset_index(name='Counts')

df_planning_fe1_val_counts = df_planning_fe1_val_counts[["fault", "fault_effect", "Counts"]]



df_planning_fe2_val_counts = df_planning_fe2.value_counts().reset_index(name='Counts')

df_planning_fe2_val_counts = df_planning_fe2_val_counts[["fault", "fault_effect", "Counts"]]
#df_planning_fe1_val_counts.concat(df_planning_fe2_val_counts)


df_planning_fe3_val_counts = df_planning_fe3.value_counts().reset_index(name='Counts')

df_planning_fe3_val_counts = df_planning_fe3_val_counts[["fault", "fault_effect", "Counts"]]

df_planning_fe4_val_counts = df_planning_fe4.value_counts().reset_index(name='Counts')

df_planning_fe4_val_counts = df_planning_fe4_val_counts[["fault", "fault_effect", "Counts"]]


df_planning_fe5_val_counts = df_planning_fe5.value_counts().reset_index(name='Counts')

df_planning_fe5_val_counts = df_planning_fe5_val_counts[["fault", "fault_effect", "Counts"]]

df_planning_fe6_val_counts = df_planning_fe6.value_counts().reset_index(name='Counts')

df_planning_fe6_val_counts = df_planning_fe6_val_counts[["fault", "fault_effect", "Counts"]]



df_final = pd.concat([df_planning_fe1_val_counts,df_planning_fe2_val_counts,df_planning_fe3_val_counts,df_planning_fe4_val_counts,df_planning_fe5_val_counts,df_planning_fe6_val_counts],axis=0)
#print(df_final)

df_final_to_plot = df_final.copy()


print(df_final_to_plot)
df_final["fault_effect"] = df_final["fault_effect"].replace(to_replace=1,value="Tool_thrown")
df_final["fault_effect"] = df_final["fault_effect"].replace(to_replace=2,value="Human_collision")
df_final["fault_effect"] = df_final["fault_effect"].replace(to_replace=3,value="back_support_collision")
df_final["fault_effect"] = df_final["fault_effect"].replace(to_replace=4,value="right_support_collision")
df_final["fault_effect"] = df_final["fault_effect"].replace(to_replace=5,value="front_support_collision")
df_final["fault_effect"] = df_final["fault_effect"].replace(to_replace=6,value="Controller_failure")

df_final["fault"] = df_final["fault"].replace(to_replace=1,value="Noise")
df_final["fault"] = df_final["fault"].replace(to_replace=2,value="Stuck_at")
df_final["fault"] = df_final["fault"].replace(to_replace=3,value="package_drop")
df_final["fault"] = df_final["fault"].replace(to_replace=4,value="offset")


print(df_final)


df_final.to_csv("/home/acefly/robot_fib/Tables/fault_effects_during_planning.csv", index=False, header=True)


fig = go.Figure(data=[go.Surface(z=df_final_to_plot["Counts"], x=df_final_to_plot["fault"], y=df_final_to_plot["fault_effect"])])

fig.update_layout(title='Fault_space')

fig.show()
#plotly.offline.plot(fig, filename='/home/acefly/robot_fib/plots/Scatter3d.html')


#print(df.value_counts())

