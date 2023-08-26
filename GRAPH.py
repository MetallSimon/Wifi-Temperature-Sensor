import pandas as pd
import plotly.express as px

df1 = pd.read_csv('C:/Users/Default/Desktop/converted0.csv')
#df2 = pd.read_csv('C:/Users/Default/Desktop/volt0.csv')

fig = px.line(df1, x = 'time_ms', y = 'message', color='device_id')
#fig.add_trace(px.line(df2, x='time_ms', y='message', color='device_id').data[0])
fig.show()
