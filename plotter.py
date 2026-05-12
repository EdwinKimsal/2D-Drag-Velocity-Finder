import math
import matplotlib.pyplot as plt

# Variables
FINAL_X = 2.432 # Meters
FINAL_Y = .4209 # Meters
ANGLE = math.radians(45) # Radians

# Constants
MASS = 0.0027 # Kilograms
BALL_RADIUS = 0.02 # Meters
AIR_DENSITY = 1.225 # kg/m^3
DRAG_COEF = 0.50
GRAVITY = 9.81 # m/s^2
TIME_DIFF = 0.005 # Difference in points
DELTA_GOAL = 0.01
DIFF_FACTOR = 0.75
MAX_ITER = 100

area = math.pi * BALL_RADIUS**2
drag_factor = (0.5 * AIR_DENSITY * DRAG_COEF * area) / MASS
init_vel = (FINAL_X / math.cos(ANGLE)) * math.sqrt(GRAVITY / (2 * (FINAL_X * math.tan(ANGLE) - FINAL_Y)))


# Get the path and distance of the object to the goal
def get_drag_path_info(vel, plot=False, color="blue", label=None):
    # Set initial point as the origin
    pos_x = 0
    pos_y = 0
    label_drawn = False
    
    # Calculate directional velocities
    vel_x = vel * math.cos(ANGLE)
    vel_y = vel * math.sin(ANGLE)
    
    while ((pos_y >= 0 or pos_y >= FINAL_Y) and pos_x < FINAL_X):
        # Calculate accelerations
        acc_x = -drag_factor * vel * vel_x
        acc_y = -GRAVITY - drag_factor * vel * vel_y

        # Update velocities
        vel_x =  vel_x + (acc_x * TIME_DIFF)
        vel_y =  vel_y + (acc_y * TIME_DIFF)

        # Update positions
        pos_x = pos_x + (vel_x * TIME_DIFF)
        pos_y = pos_y + (vel_y * TIME_DIFF)

        # Plot if needed
        if plot:
            if label and not label_drawn:
                plt.scatter(pos_x, pos_y, color=color, label=label)
                label_drawn = True
            else:
                plt.scatter(pos_x, pos_y, color=color)

    # Return the final distance
    dist = FINAL_Y - pos_y
    return dist


# Get the path of the object to the goal
def get_reg_path_info(vel, plot=False, color="lime", label=None):
    # Set initial point as the origin
    pos_x = 0
    pos_y = 0
    label_drawn = False

    # Calculate directional velocities
    vel_x = vel * math.cos(ANGLE)
    vel_y = vel * math.sin(ANGLE)

    while pos_x < FINAL_X and pos_y >= 0:
        # Calculate accelerations without air drag
        acc_x = 0
        acc_y = -GRAVITY

        # Update velocities
        vel_x = vel_x + (acc_x * TIME_DIFF)
        vel_y = vel_y + (acc_y * TIME_DIFF)

        # Update positions
        pos_x = pos_x + (vel_x * TIME_DIFF)
        pos_y = pos_y + (vel_y * TIME_DIFF)

        # Plot if needed
        if plot:
            if label and not label_drawn:
                plt.scatter(pos_x, pos_y, color=color, label=label)
                label_drawn = True
            else:
                plt.scatter(pos_x, pos_y, color=color)
    
    return


# Update the velocity of the shot
def update_shot(vel, dist):
    vel = vel + (dist * DIFF_FACTOR)
    return vel  


# Use functions
def main():  
    vel = init_vel
    dist = math.inf

    # Plot the ideal scenario path
    get_reg_path_info(vel, True, "lime", "No drag")
    
    for i in range(MAX_ITER):
        dist = get_drag_path_info(vel)
        if (abs(dist) > DELTA_GOAL):
            vel = update_shot(vel, dist)
        else:
            break

    # Plot the air resistance path
    get_drag_path_info(vel, True, "blue", "Quadratic drag")

    # Plot the goal
    plt.scatter(FINAL_X, FINAL_Y, color="red", label="Target")

    # Initialize graph and show it
    plt.title("Path Traveled")
    plt.xlabel("X Distance")
    plt.ylabel("Y Distance")
    plt.gca().set_aspect("equal", adjustable="box")
    plt.margins(0.20)
    plt.legend(fontsize="xx-small", loc="upper right")
    plt.show()


main()
