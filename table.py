import random

import pybullet as p
import time
import pybullet_data
import numpy as np

# from filterpy.kalman import KalmanFilter
# from copy import deepcopy

physicsClient = p.connect(p.GUI)  # Setup GUI

# !!!! Assume there is no air resistence !!!!
# ______________________________Set environment__________________________________

g = 9.81
STEP_SIZE = 1. / 240.
p.setGravity(0, 0, -g)

# Get path to load resources
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the ground plane
planeId = p.loadURDF("plane.urdf")

# ______________________________Loading in table__________________________________
# Setting the table's position
tableStartPos = [0, 0, 0]
tableStartOrientation = p.getQuaternionFromEuler([0, 0, 0])

# Table tennis table dimensions
table_length = 2.74  # meters
table_width = 1.525  # meters
table_height = 0.76  # meters

table_color = [0, 0, 0.8, 1]

# Dimensions for the legs
leg_height = table_height
leg_width = 0.05  # 5 cm
leg_length = 0.05  # 5 cm

# Half extents for the tabletop and legs
table_half_extents = [table_length / 2, table_width / 2, 0.02]
leg_half_extents = [leg_length / 2, leg_width / 2, leg_height / 2]

# Adding friction coefficient
table_friction = 0.5

# Adjusted base position for the tabletop to be on top of the legs
table_base_position = [0, 0, leg_height + 0.02 / 2]

# Create a table using a box shape
table_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[table_length / 2, table_width / 2, table_height / 20])
table_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[table_length / 2, table_width / 2, table_height / 20],
                                   rgbaColor=table_color)
tableId = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=table_collision, baseVisualShapeIndex=table_visual,
                            basePosition=table_base_position)


# Function to add a leg to the table
def add_table_leg(base_pos):
    leg_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=leg_half_extents)
    leg_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=leg_half_extents, rgbaColor=[0.5, 0.3, 0.2, 1])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=leg_collision, baseVisualShapeIndex=leg_visual,
                      basePosition=base_pos)


# Positions of the four legs relative to the table center
leg_positions = [
    [-table_length / 2 + leg_length / 2, -table_width / 2 + leg_width / 2, leg_height / 2],
    [table_length / 2 - leg_length / 2, -table_width / 2 + leg_width / 2, leg_height / 2],
    [-table_length / 2 + leg_length / 2, table_width / 2 - leg_width / 2, leg_height / 2],
    [table_length / 2 - leg_length / 2, table_width / 2 - leg_width / 2, leg_height / 2],
]

# Add legs to the table
for position in leg_positions:
    add_table_leg(position)

# Adjust the table's dynamics, especially its restitution
tableRestitution = 0.9  # Setting a high restitution for the table to improve bounce
p.changeDynamics(tableId, -1, restitution=tableRestitution, lateralFriction=table_friction)


# Function to add boundary lines as thin boxes
def add_boundary_line(base_pos, half_extents, color=None):
    if color is None:
        color = [1, 1, 1, 1]
    line_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    line_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
    line_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=line_collision, baseVisualShapeIndex=line_visual,
                                basePosition=base_pos)
    p.changeDynamics(line_id, -1, restitution=tableRestitution, lateralFriction=table_friction)


# line dimensions and positions
line_thickness = 0.02
line_offset = 0.001

# End lines
end_line_half_extents = [line_thickness / 2, (table_width / 2) - (line_thickness / 2), line_offset]
# Side lines
side_line_half_extents = [(table_length / 2) - (line_thickness / 2), line_thickness / 2, line_offset]
# Center line
center_line_half_extents = [(table_length / 2) - (line_thickness / 2), line_thickness / 2, line_offset]

# Adjusted positions to ensure lines do not extend out of the table
add_boundary_line([table_length / 2 - line_thickness / 2, 0, table_height + 0.028 + 0.02 + line_offset],
                  end_line_half_extents)  # Right end line
add_boundary_line([-table_length / 2 + line_thickness / 2, 0, table_height + 0.028 + 0.02 + line_offset],
                  end_line_half_extents)  # Left end line
add_boundary_line([0, table_width / 2 - line_thickness / 2, table_height + 0.028 + 0.02 + line_offset],
                  side_line_half_extents)  # Top side line
add_boundary_line([0, -table_width / 2 + line_thickness / 2, table_height + 0.028 + 0.02 + line_offset],
                  side_line_half_extents)  # Bottom side line
# Center line placed correctly on the table
add_boundary_line([0, 0, table_height + 0.028 + 0.02 + line_offset], center_line_half_extents)  # Corrected center line

# ______________________________Loading in net__________________________________
# Net dimensions
net_height = 0.1525  # meters
net_thickness = 0.02  # meters
net_color = [1, 1, 1, 1]  # White color for the net

net_half_extents = [net_thickness / 2, table_width / 2, net_height / 2]

net_friction = 0.6
net_base_position = [0, 0, table_height + 0.02 + net_height / 2]

# Create the net
net_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=net_half_extents)
net_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=net_half_extents, rgbaColor=net_color)
netId = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=net_collision, baseVisualShapeIndex=net_visual,
                          basePosition=net_base_position)
p.changeDynamics(netId, -1, lateralFriction=net_friction, restitution=0.1)

# ______________________________Loading in ball__________________________________
# Load a ping pong ball and set its position
ball_mass = 0.0027
ball_radius = 0.02
ball_collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_radius)
ball_visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=ball_radius, rgbaColor=[1, 1, 1, 1])
inertia = (2 / 3) * ball_mass * (ball_radius ** 2)

# ballStartPos = [-table_length / 2, table_width / 2, table_height + 0.5]
# ballStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
# ball_id = p.createMultiBody(ball_mass, ball_collision_shape, ball_visual_shape, ballStartPos)
# p.changeDynamics(ball_id, -1, restitution=0.9, mass=ball_mass,
#                  localInertiaDiagonal=[inertia, inertia, inertia],
#                  spinningFriction=0.001, rollingFriction=0.001, lateralFriction=0.2)

# ______________________________Loading in racket__________________________________
'''
NOTE: for now, let the racket be a thin plank for simplicity
'''
racket_mass = 0  # mass is 0 to let the racket "float" over the table
racket_length = 0.25  # Length of the racket in meters
racket_width = 0.15  # Width of the racket in meters
racket_thickness = 0.025  # Thickness of the racket in meters
racket_restitution = 0.9  # Bounce coefficient
racket_friction = 0.2  # Friction coefficient


def add_racket(racket_position, orientation):
    quaternion_orientation = p.getQuaternionFromEuler(orientation)

    racket_collision = p.createCollisionShape(p.GEOM_BOX,
                                              halfExtents=[racket_length / 2, racket_width / 2, racket_thickness / 2])
    racket_visual = p.createVisualShape(p.GEOM_BOX,
                                        halfExtents=[racket_length / 2, racket_width / 2, racket_thickness / 2],
                                        rgbaColor=[1, 0, 0, 1])
    racket_id = p.createMultiBody(baseMass=racket_mass, baseCollisionShapeIndex=racket_collision,
                                  baseVisualShapeIndex=racket_visual, basePosition=racket_position,
                                  baseOrientation=quaternion_orientation)
    p.changeDynamics(racket_id, -1, restitution=racket_restitution, lateralFriction=racket_friction)
    return racket_id


# ______________________________Trajectory__________________________________

def calculate_trajectory(initial_position, initial_velocity, flight_time):
    """
    Calculate the trajectory of a projectile given its initial position and velocity.


    """


    time_step = STEP_SIZE  # Time step for the simulation (seconds)

    positions = []  # List to store the trajectory points
    position = np.array(initial_position)
    velocity = np.array(initial_velocity)

    while True:
        # this is a first-order approximation, try to make a simple numerical example (c.f. DSE Physics)
        # and discretize the process
        # this scheme can handle air resistance (i.e. drift term in ODE)
        # Update position based on the current velocity
        position += velocity * time_step
        # Update the z-component of the velocity due to gravity
        velocity[2] += (-g) * time_step
        position[2] += 0.5 * (-g) * time_step ** 2
        # velocity[2] = velocity[2] * flight_time + 0.5 * (-g) * flight_time**2
        # Add the new position to the trajectory list
        positions.append(tuple(position))

        # Check if the ball has hit the table or gone out of bounds
        if position[2] <= table_height or abs(position[0]) > table_length / 2 or abs(position[1]) > table_width / 2:
            break

    # s = ut + 0.5 * -g * t**2
    # time_of_flight = 3
    # to-do = 0
    temp = np.array([(velocity[0] * t, velocity[1] * t, velocity[2] * t + 0.5 * -g * t ** 2) for t in
                     np.arange(0, flight_time, time_step)])
    # apply a sanity check for hit table or out of table

    return positions


def shoot_ball_and_predict_trajectory(traj_ball_id):
    force_magnitude = random.uniform(2, 3)
    force_direction = np.array([1.5, random.uniform(-0.2, 0.2), force_magnitude])  # Adjust for different trajectories

    # use F = m*dv/dt
    initial_velocity = (force_direction / ball_mass) * STEP_SIZE
    # print(f"velocity: {initial_velocity}")
    # Get the initial position of the ball
    initial_position, _ = p.getBasePositionAndOrientation(ball_id)

    # Calculate flight time based on the vertical component of the velocity
    # Time to reach the peak height
    time_to_peak = abs(initial_velocity[2]) / g
    # Total flight time (up and down)
    total_flight_time = 2 * time_to_peak

    # print(f"Estimated flight time: {total_flight_time} seconds")

    # Calculate the trajectory
    trajectory = calculate_trajectory(initial_position, initial_velocity, total_flight_time)
    # print(trajectory)

    for i in range(len(trajectory) - 1):
        p.addUserDebugLine(trajectory[i], trajectory[i + 1], [1, 0, 0], lifeTime=2)

    # Apply the force to the ball in PyBullet
    p.applyExternalForce(traj_ball_id, -1, force_direction, [0, 0, 0], p.WORLD_FRAME)
    return trajectory


# ______________________________Ball Simulation__________________________________
def reset_ball():
    x_pos = -table_length / 2 + random.uniform(0.1,
                                               table_length / 4)  # random.uniform(-table_length / 4, -table_length / 8)
    y_pos = random.uniform(-table_width / 2,
                           table_width / 2)  # random.uniform(-table_width / 2 + 0.1, table_width / 2 - 0.1)
    z_pos = table_height + 0.5  # table_height + ball_radius + 0.1

    ball_position = [x_pos, y_pos, z_pos]
    ball_orientation = p.getQuaternionFromEuler([0, 0, 0])

    ball_reset_id = p.createMultiBody(ball_mass, ball_collision_shape, ball_visual_shape, ball_position,
                                      ball_orientation)
    p.changeDynamics(ball_reset_id, -1, restitution=0.9, mass=ball_mass,
                     localInertiaDiagonal=[inertia, inertia, inertia],
                     spinningFriction=0.001, rollingFriction=0.001, lateralFriction=0.2)

    return ball_reset_id


def shoot_ball(shoot_ball_id):
    force_magnitude = random.uniform(2, 3)  # Adjust force magnitude as needed for variability
    force_direction = [1.5, 0, force_magnitude]  # Adjust for different trajectories
    p.applyExternalForce(shoot_ball_id, -1, force_direction, [0, 0, 0], p.WORLD_FRAME)


# ______________________________Ball Logic__________________________________
def is_ball_oob(oob_ball_id):
    ball_pos, _ = p.getBasePositionAndOrientation(oob_ball_id)
    return ball_pos[2] < table_height - 0.1 or abs(ball_pos[0]) > table_length / 2 or abs(ball_pos[1]) > table_width / 2


def is_ball_stopped(stopped_ball_id):
    threshold = 0.1
    velocity, _ = p.getBaseVelocity(stopped_ball_id)
    total_velocity = sum([abs(v) for v in velocity])  # Calculate the total velocity (magnitude)
    return total_velocity < threshold


def is_ball_crossed_net(net_ball_id):
    ball_pos, _ = p.getBasePositionAndOrientation(net_ball_id)
    return ball_pos[0] > 0


# ______________________________Racket things__________________________________
def find_intercept_point(trajectory):
    for point in trajectory:
        if table_height < point[2] and point[0] > table_length * 0.2:
            print(point)
            return point
    return None  # If no suitable point is found


def teleport_racket_to_intercept(intercept_point, ball_id, racket_id):
    ball_pos, _ = p.getBasePositionAndOrientation(ball_id)

    # Calculate racket position for intercept; adjust y-position for racket dimensions and desired contact point
    racket_position = [intercept_point[0], intercept_point[1], intercept_point[
        2] - 0.2]
    racket_orientation = [0, np.pi * 0.75, 0]

    p.resetBasePositionAndOrientation(racket_id, racket_position, p.getQuaternionFromEuler(racket_orientation))
    print("racket created")

    return racket_id


# ______________________________Keyboard press__________________________________
from pynput import keyboard

is_s_key_pressed = False


def on_press(key):
    global is_s_key_pressed

    try:
        if key.char == 's':
            is_s_key_pressed = not is_s_key_pressed
    except AttributeError:
        # Handle special key press here if needed
        pass


listener = keyboard.Listener(on_press=on_press)
listener.start()
# ______________________________Start simulation__________________________________
while True:
    if is_s_key_pressed:
        ball_id = reset_ball()
        trajectory = shoot_ball_and_predict_trajectory(ball_id)
        point = find_intercept_point(trajectory)
        racket_id = add_racket([table_length / 2, 0, table_height], [0, np.pi * 0.75, 0])  # Add racket at new position
        teleport_racket_to_intercept(point, ball_id, racket_id)
        # Simulation loop
        for i in range(100000):
            p.stepSimulation()
            time.sleep(1. / 240.)

            # Logic to check if the ball is out of the table
            if is_ball_oob(ball_id) or is_ball_stopped(ball_id):
                p.removeBody(ball_id)  # Remove the current ball
                ball_id = reset_ball()  # Reset the ball for the next shot
                trajectory = shoot_ball_and_predict_trajectory(ball_id)  # Shoot again
                point = find_intercept_point(trajectory)
                teleport_racket_to_intercept(point, ball_id, racket_id)
                continue
        p.disconnect()
