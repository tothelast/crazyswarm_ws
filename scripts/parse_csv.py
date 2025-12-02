import csv

filename = 'logs/experiment_data_20251202_153401.csv'

with open(filename, 'r') as f:
    reader = csv.reader(f)
    header = next(reader)
    row1 = next(reader)

print(f"Header length: {len(header)}")
print(f"Row length: {len(row1)}")

for h, v in zip(header, row1):
    print(f"{h}: {v}")
