import matplotlib.pyplot as plt

# Function to read waypoints from a file
def read_waypoints(filename):
    waypoints = []
    with open(filename, 'r') as file:
        for line in file:
            line = line.strip()  # Remove whitespace characters
            if line:  # Check if the line is not empty
                try:
                    latitude, longitude = map(float, line.split(','))
                    waypoints.append((latitude, longitude))
                except ValueError:
                    print(f"Skipping malformed line: {line}")
    return waypoints

# Function to plot waypoints as a path with a specified color and label
def plot_path(waypoints, color, label):
    if not waypoints:
        print(f"No waypoints to plot for {label}.")
        return
    
    latitudes = [point[0] for point in waypoints]
    longitudes = [point[1] for point in waypoints]

    # Plot the points with specific color and label
    plt.plot(longitudes, latitudes, marker='o', markersize=0, linestyle='-', color=color, label=label)


