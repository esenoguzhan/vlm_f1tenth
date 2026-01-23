import csv
import argparse
import os
import math
import matplotlib.pyplot as plt

def shift_raceline(input_path, output_path, shift_amount, plot=True):
    """
    Shifts the raceline perpendicular to the path direction.
    +shift_amount = Left
    -shift_amount = Right
    """
    
    # 1. Read Data
    data = []
    try:
        with open(input_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 2:
                    data.append([float(row[0]), float(row[1]), float(row[2]) if len(row) > 2 else 0.0])
    except FileNotFoundError:
        print(f"Error: File {input_path} not found.")
        return

    if not data:
        print("No data found.")
        return

    shifted_data = []

    # 2. Process and Shift
    for i in range(len(data)):
        # Get current, prev, next points for tangent calculation
        curr_p = data[i]
        prev_p = data[i-1] # Wraps around for index 0
        next_p = data[(i+1) % len(data)]

        # Calculate tangent vectors
        # Simple finite difference
        dx = next_p[0] - prev_p[0]
        dy = next_p[1] - prev_p[1]
        
        # Heading angle
        yaw = math.atan2(dy, dx)

        # Perpendicular shift (Normal vector)
        # Shift Left: x' = x - d*sin(yaw), y' = y + d*cos(yaw)
        # Shift Right: Opposite
        
        # Let's say vector is (cos, sin)
        # Normal Left is (-sin, cos)
        
        new_x = curr_p[0] - shift_amount * math.sin(yaw)
        new_y = curr_p[1] + shift_amount * math.cos(yaw)
        
        shifted_data.append([new_x, new_y, curr_p[2]])

    # 3. Save Output
    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(shifted_data)
    
    print(f"Saved shifted raceline to {output_path} (Shift: {shift_amount}m)")
    
    # 4. Plot Comparison
    if plot:
        orig_x = [d[0] for d in data]
        orig_y = [d[1] for d in data]
        new_x = [d[0] for d in shifted_data]
        new_y = [d[1] for d in shifted_data]
        
        plt.figure(figsize=(10, 10))
        plt.plot(orig_x, orig_y, 'r--', label='Original', alpha=0.5)
        plt.plot(new_x, new_y, 'b-', label=f'Shifted ({shift_amount}m)')
        plt.legend()
        plt.title(f"Raceline Shift Comparison\nOriginal: {input_path}")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.axis('equal')
        plt.grid(True)
        plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Shift raceline waypoints laterally.')
    parser.add_argument('file', help='Input CSV file')
    parser.add_argument('shift', type=float, help='Shift amount in meters (Positive = Left)')
    parser.add_argument('--out', help='Output file name', default='shifted_raceline.csv')
    parser.add_argument('--no-plot', action='store_true', help='Disable plotting')
    
    args = parser.parse_args()
    
    shift_raceline(args.file, args.out, args.shift, plot=not args.no_plot)
