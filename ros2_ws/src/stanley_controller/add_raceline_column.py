import csv
import os

# Define file paths
base_dir = os.path.dirname(os.path.abspath(__file__))
file_path = os.path.join(base_dir, 'racelines', 'raceline_small.csv')
temp_file_path = os.path.join(base_dir, 'racelines', 'raceline_small_temp.csv')

print(f"Processing {file_path}...")

if not os.path.exists(file_path):
    print(f"Error: {file_path} not found.")
    exit(1)

with open(file_path, 'r') as infile, open(temp_file_path, 'w', newline='') as outfile:
    reader = csv.reader(infile)
    writer = csv.writer(outfile)
    
    # Read and skip header
    try:
        header = next(reader)
        print("Removed header:", header)
    except StopIteration:
        print("Error: File is empty")
        exit(1)
        
    count = 0
    # Write data with added column
    for row in reader:
        if row: # Avoid empty lines if any
            # Clean whitespace just in case
            row = [x.strip() for x in row]
            row.append(4.0)
            writer.writerow(row)
            count += 1

# Replace original file
os.replace(temp_file_path, file_path)
print(f"Successfully processed {count} rows in {file_path}")
