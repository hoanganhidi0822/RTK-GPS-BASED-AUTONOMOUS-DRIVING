import numpy as np
import matplotlib.pyplot as plt
import math
import matplotlib.patches as patches

# Vehicle parameters
LENGTH = 2.0  # total vehicle length
WIDTH = 1.0   # total vehicle width
BACKTOWHEEL = 0.2  # distance from rear to vehicle center
WHEEL_LEN = 0.3  # wheel length
WHEEL_WIDTH = 0.2  # wheel width
TREAD = 0.5  # width between left and right wheels
WB = 1.8  # wheel base: distance between front and rear axles

def plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover
    """
    Plots a car at the given position and orientation.
    :param x: X-coordinate of the car's rear axle center.
    :param y: Y-coordinate of the car's rear axle center.
    :param yaw: Orientation of the car in radians.
    :param steer: Steering angle in radians.
    :param cabcolor: Color of the car body.
    :param truckcolor: Color of the wheels.
    """
    steer = -steer
    # Define the car outline and wheel shapes
    outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, -WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    # Rotation matrices
    Rot1 = np.array([[math.cos(yaw),    math.sin(yaw)],
                     [-math.sin(yaw),   math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer),  math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    # Transform front wheels based on steering angle
    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    # Apply vehicle yaw rotation
    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T
    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    # Translate to vehicle position
    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    # Plot car body and wheels
    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), cabcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "*")  # Mark the vehicle's position
def plot_obstacles(obstacles, width=1, height=2):
    """
    Plot obstacles as rectangles.
    """
    ax = plt.gca()
    for obs in obstacles:
        x, y = obs
        rect = patches.Rectangle((x - width / 2, y - height / 2), width, height,
                                  linewidth=1, edgecolor='r', facecolor='none')
        ax.add_patch(rect)
        
# def visualize():
#     plt.cla()
#     plt.plot(tx, ty, "--y", label="Target Course" )

#     tyaw = np.array(tyaw)
#     # Tính toán đường song song
#     left_x = tx + (MAX_ROAD_WIDTH/2 +0.5) * np.cos(tyaw + np.pi / 2)  # Dịch chuyển 3m sang bên trái
#     left_y = ty + (MAX_ROAD_WIDTH/2 +0.5) * np.sin(tyaw + np.pi / 2)
#     right_x = tx +( MAX_ROAD_WIDTH/2+0.5) * np.cos(tyaw - np.pi / 2)  # Dịch chuyển 3m sang bên phải
#     right_y = ty +( MAX_ROAD_WIDTH/2+0.5) * np.sin(tyaw - np.pi / 2)

#     # Vẽ đường biên trái và phải
#     plt.plot(left_x, left_y, "-b", label="Left Boundary")
#     plt.plot(right_x, right_y, "-b", label="Right Boundary")
    
#     plt.plot(ob[:, 0], ob[:, 1], "xb", label="Obstacles")

#     # Plot
#     for traj in paths:
#         plt.plot(traj.x, traj.y, "black", alpha=0.05)

#     plt.plot(optimal_path.x[1:], optimal_path.y[1:], "-g",alpha=0.95, label="Optimal Trajectory")
#     circle = plt.Circle((x, y), lookahead_distance, color='b', fill=False, linestyle='--')
#     plt.gca().add_artist(circle)
#     plt.scatter(lookahead_point[0], lookahead_point[1], color='red', s=20, label="Lookahead Point")
    
#     plot_car(x, y, yaw, np.deg2rad(steering_angle), cabcolor="-r")
#     plt.scatter(x, y, color='blue', label="Current Position")

#     # Plot the projected point
#     if projected_point is not None:
#         plt.scatter(projected_point[0], projected_point[1], color='green', s=20, label="Projected Point")

#     plt.xlim(x - (area - 5), x + (area - 5))
#     plt.ylim(y - (area - 10), y + (area + 10))
#     plt.title(f"Speed: {car_speed * 3.6:.2f} km/h | Steering Angle: {(car_steer):.2f}°")
#     plt.grid(True)
    
#     cv2.imshow("img", image)
#     plt.pause(1/1000)

import numpy as np
import matplotlib.pyplot as plt
import cv2
from matplotlib.lines import Line2D

def calculate_boundaries(tx, ty, tyaw, max_road_width):
    """Calculate the left and right boundaries for the road."""
    offset = (max_road_width / 2 + 0.5)
    cos_tyaw = np.cos(tyaw)
    sin_tyaw = np.sin(tyaw)
    
    left_x = tx + offset * (cos_tyaw + np.pi / 2)
    left_y = ty + offset * (sin_tyaw + np.pi / 2)
    right_x = tx + offset * (cos_tyaw - np.pi / 2)
    right_y = ty + offset * (sin_tyaw - np.pi / 2)
    
    return left_x, left_y, right_x, right_y

def plot_target_course(tx, ty):
    """Plot the target course with yellow dashed line."""
    plt.plot(tx, ty, "--y", label="Target Course")

def plot_boundaries(left_x, left_y, right_x, right_y):
    """Plot the left and right boundaries."""
    plt.plot(left_x, left_y, "-b", label="Left Boundary")
    plt.plot(right_x, right_y, "-b", label="Right Boundary")

def plot_obstacles(ob):
    """Plot the obstacles."""
    plt.plot(ob[:, 0], ob[:, 1], "xb", label="Obstacles")

def plot_paths(paths):
    """Plot the paths."""
    for traj in paths:
        plt.plot(traj.x, traj.y, "black", alpha=0.05)

def plot_optimal_path(optimal_path):
    """Plot the optimal path."""
    plt.plot(optimal_path.x[1:], optimal_path.y[1:], "-g", alpha=0.95, label="Optimal Trajectory")

def plot_lookahead(x, y, lookahead_distance, lookahead_point):
    """Plot the lookahead circle and point."""
    circle = plt.Circle((x, y), lookahead_distance, color='b', fill=False, linestyle='--')
    plt.gca().add_artist(circle)
    plt.scatter(lookahead_point[0], lookahead_point[1], color='red', s=20, label="Lookahead Point")

def plot_car_and_position(x, y, yaw, steering_angle, plot_car):
    """Plot the car and its current position."""
    plot_car(x, y, yaw, np.deg2rad(steering_angle), cabcolor="-r")
    plt.scatter(x, y, color='blue', label="Current Position")

def plot_projected_point(projected_point):
    """Plot the projected point if provided."""
    if projected_point is not None:
        plt.scatter(projected_point[0], projected_point[1], color='green', s=20, label="Projected Point")

def update_plot_area(x, y, area):
    """Set plot limits based on the car's position."""
    plt.xlim(x - (area - 5), x + (area - 5))
    plt.ylim(y - (area - 10), y + (area + 10))

def update_title(car_speed, car_steer):
    """Update the title with the car's speed and steering angle."""
    plt.title(f"Speed: {car_speed * 3.6:.2f} km/h | Steering Angle: {car_steer:.2f}°")

def Visualize(tx, ty, tyaw, ob, paths, optimal_path, x, y, yaw, 
              steering_angle, lookahead_distance, lookahead_point, 
              projected_point, car_speed, car_steer, area, 
              MAX_ROAD_WIDTH, image, plot_car):
    """Plots a simulation frame with the given parameters."""
    plt.cla()

    # Pre-calculate static values
    tyaw = np.array(tyaw)
    left_x, left_y, right_x, right_y = calculate_boundaries(tx, ty, tyaw, MAX_ROAD_WIDTH)

    # Plot various elements
    plot_target_course(tx, ty)
    plot_boundaries(left_x, left_y, right_x, right_y)
    plot_obstacles(ob)
    plot_paths(paths)
    plot_optimal_path(optimal_path)
    plot_lookahead(x, y, lookahead_distance, lookahead_point)
    plot_car_and_position(x, y, yaw, steering_angle, plot_car)
    plot_projected_point(projected_point)

    # Update plot area and title
    update_plot_area(x, y, area)
    update_title(car_speed, car_steer)

    # Display the image using OpenCV
    cv2.imshow("img", image)

    # Pause to update the plot
    plt.pause(1 / 1000)
