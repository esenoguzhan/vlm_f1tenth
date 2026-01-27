import csv
import matplotlib.pyplot as plt
import argparse
import os

def visualize_raceline(csv_path):
    x = []
    y = []
    v = []

    try:
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 3:
                    x.append(float(row[0]))
                    y.append(float(row[1]))
                    v.append(float(row[2]))
    except FileNotFoundError:
        print(f"Error: File {csv_path} not found.")
        return

    if not x:
        print("No data found in file.")
        return

    plt.figure(figsize=(10, 8))
    
    # Plot the path, colored by velocity
    sc = plt.scatter(x, y, c=v, cmap='viridis', s=10, label='Waypoints')
    plt.colorbar(sc, label='Velocity (m/s)')
    
    # Mark Start and End
    plt.plot(x[0], y[0], 'go', markersize=10, label='Start')
    plt.plot(x[-1], y[-1], 'rx', markersize=10, label='End')
    
    # Plot line connecting points to show order
    plt.plot(x, y, 'k-', alpha=0.3, linewidth=1)

    plt.title(f'Raceline Visualization: {os.path.basename(csv_path)}')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    # Default to min_curve.csv in the same directory if no argument provided
    default_path = os.path.join(os.path.dirname(__file__), 'min_curve.csv')
    
    # Check if a specific file was passed
    parser = argparse.ArgumentParser(description='Visualize raceline waypoints.')
    parser.add_argument('file', nargs='?', default=default_path, help='Path to the .csv file')
    
    args = parser.parse_args()
    
    print(f"Visualizing: {args.file}")
    visualize_raceline(args.file)
