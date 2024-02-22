import random

import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)  # Setup GUI

# ______________________________Set environment__________________________________
p.setGravity(0, 0, -9.81)

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
ballStartPos = [-table_length / 2, table_width / 2, table_height + 0.5]
ballStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
ball_mass = 0.0027
ball_radius = 0.02
ball_collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_radius)
ball_visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=ball_radius, rgbaColor=[1, 1, 1, 1])
ball_id = p.createMultiBody(ball_mass, ball_collision_shape, ball_visual_shape, ballStartPos)

inertia = (2 / 3) * ball_mass * (ball_radius ** 2)

p.changeDynamics(ball_id, -1, restitution=0.9, mass=ball_mass,
                 localInertiaDiagonal=[inertia, inertia, inertia],
                 spinningFriction=0.001, rollingFriction=0.001, lateralFriction=0.2)

# ______________________________Ball Simulation__________________________________
def reset_ball():
    x_pos = random.uniform(-table_length / 4, -table_length / 8)
    y_pos = random.uniform(-table_length / 4, -table_length / 8)
    z_pos = random.uniform(-table_length / 4, -table_length / 8)
# Shoot the ping pong ball by applying a force
ballForce = [2.5, -1.3, 0.7]
p.applyExternalForce(objectUniqueId=ball_id, linkIndex=-1, forceObj=ballForce, posObj=ballStartPos, flags=p.WORLD_FRAME)

# ______________________________Start simulation__________________________________
# Simulation loop
for i in range(100000):
    p.stepSimulation()
    time.sleep(1. / 240.)

p.disconnect()
