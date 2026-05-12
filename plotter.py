import math
import matplotlib.pyplot as plt

# Variables
FINAL_X = 5 # Meters
FINAL_Y = .5 # Meters
ANGLE = math.radians(45) # Radians

# Constants
MASS = 0.040 # Kilograms
BALL_RADIUS = 0.002 # Meters
AIR_DENSITY = 1.225 # kg/m^3
DRAG_COEF = 0.40
GRAVITY = 9.81 # m/s^2
TIME_DIFF = 0.010 # Difference in points
DELTA_GOAL = 0.001
DIFF_FACTOR = 0.25

area = 2 * math.pi * BALL_RADIUS
terminal_velocity = math.sqrt((2* MASS * GRAVITY) / (area * AIR_DENSITY * DRAG_COEF))
init_vel = (FINAL_X / math.cos(ANGLE)) * math.sqrt(GRAVITY / (2 * (FINAL_X * math.tan(ANGLE) - FINAL_Y)))


def get_point_reg(time, v0_x, v0_y):
    del_x = v0_x * time
    del_y = v0_y * time - 0.5 * GRAVITY * (time**2)

    return {"del_x": del_x, "del_y": del_y}


def get_point_air(time, v0_x, v0_y):
    del_x = (terminal_velocity**2 / GRAVITY) * math.log(1 + (GRAVITY / terminal_velocity**2) * v0_x * time)

    try:
        del_y = (terminal_velocity**2 / GRAVITY) * math.log((math.cos(math.atan(v0_y / terminal_velocity) - (GRAVITY / terminal_velocity) * time)) / (math.cos(math.atan(v0_y / terminal_velocity))))
    
    except ValueError:
        del_y = (terminal_velocity**2 / GRAVITY) * math.log((math.cosh(math.atanh(v0_y / terminal_velocity) + (GRAVITY / terminal_velocity) * time)) / (math.cosh(math.atan(v0_y / terminal_velocity))))

    return {"del_x": del_x, "del_y": del_y}



def plot_reg():
    # Init function vars
    curr_x = 0
    curr_y = 0
    prev_x = 0
    prev_y = 0
    time = 0

    # Find needed vars
    v0_x = init_vel * math.cos(ANGLE)
    v0_y = init_vel * math.sin(ANGLE)

    # Iterate until it reaches the goal x position
    while True:
        points = get_point_reg(time, v0_x, v0_y)

        curr_x = points["del_x"]
        curr_y = points["del_y"]
        plt.scatter(curr_x, curr_y, color="lime")

        if curr_x >= FINAL_X:
            plt.scatter(curr_x, curr_y, color="lime")
            break

        time += TIME_DIFF


def plot_air(vel):
    # Init function vars
    curr_x = 0
    curr_y = 0
    time = 0

    # Find needed vars
    v0_x = vel * math.cos(ANGLE)
    v0_y = vel * math.sin(ANGLE)

    # Iterate until it reaches the goal x position
    while True:
        points = get_point_air(time, v0_x, v0_y)

        curr_x = points["del_x"]
        curr_y = points["del_y"]
        plt.scatter(curr_x, curr_y, color="blue")

        if curr_x >= FINAL_X:
            plt.scatter(curr_x, curr_y, color="blue")
            break

        time += TIME_DIFF

    dist = FINAL_Y - curr_y
    return {"vel": vel, "dist": dist}


def update_shot(shot_info):
    if abs(shot_info["dist"]) <= DELTA_GOAL:
        pass
    else:
        shot_info["vel"] += shot_info["dist"] * DIFF_FACTOR
    
    return shot_info    


def main():  
    shot_info = {
        "vel": init_vel,
        "dist": math.inf
    }

    while abs(shot_info["dist"]) > DELTA_GOAL:
        # Plot the ideal scenario path
        plot_reg()
        
        # Plot the air resistance path
        shot_info = plot_air(shot_info["vel"])

        # Plot the goal
        plt.scatter(FINAL_X, FINAL_Y, color="red")

        # Initialize graph and show it  
        plt.title("Path Traveled")
        plt.xlabel("X Distance")
        plt.ylabel("Y Distance")
        plt.show()

        shot_info = update_shot(shot_info)


main()
