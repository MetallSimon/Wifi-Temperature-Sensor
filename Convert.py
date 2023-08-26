import csv
from datetime import datetime

filename = 'C:/Users/Default/Desktop/log000.txt'
output_file = 'C:/Users/Default/Desktop/converted0.csv'

with open(filename) as f:
    reader = csv.reader(f)
    # Skip the first row
    next(reader)
    rows = []
    for row in reader:
        try:
            timestamp = float(row[0])
            timestamp = int(timestamp)
            date = datetime.fromtimestamp(timestamp)
            devid = int(row[1])
            Value = int(row[2])
            if Value == 0:
                Value = 20000
            Temp = ((Value/65535)*200)-50
            rows.append([date,Temp,devid])
        except:
            # Skip rows that contain text
            continue

with open(output_file, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(rows)
