from flightsql import FlightSQLClient
import os
query = """SELECT *
FROM 'census'
WHERE time >= now() - interval '24 hours'
AND ('bees' IS NOT NULL OR 'ants' IS NOT NULL)"""

# Define the query client
query_client = FlightSQLClient(
  host = "eu-central-1-1.aws.cloud2.influxdata.com",
  token = os.environ.get("INFLUXDB_TOKEN"),
  metadata={"bucket-name": "data"})

# Execute the query
info = query_client.execute(query)
reader = query_client.do_get(info.endpoints[0].ticket)

# Convert to dataframe
data = reader.read_all()
df = data.to_pandas().sort_values(by="time")
print(df)
