# import random
#
# import pybullet as p
# import time
# import pybullet_data
# import numpy as np
# from filterpy.kalman import KalmanFilter
# from copy import deepcopy
#
# physicsClient = p.connect(p.GUI)  # Setup GUI
#
# # !!!! Assume there is no air resistence !!!!
# # ______________________________Set environment__________________________________
# p.setGravity(0, 0, -9.81)
#
# # Get path to load resources
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
#
# # Load the ground plane
# planeId = p.loadURDF("plane.urdf")
#
# # ______________________________Loading in table__________________________________
# # Setting the table's position
# tableStartPos = [0, 0, 0]
# tableStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
#
# # Table tennis table dimensions
# table_length = 2.74  # meters
# table_width = 1.525  # meters
# table_height = 0.76  # meters
#
# table_color = [0, 0, 0.8, 1]
#
# # Dimensions for the legs
# leg_height = table_height
# leg_width = 0.05  # 5 cm
# leg_length = 0.05  # 5 cm
#
# # Half extents for the tabletop and legs
# table_half_extents = [table_length / 2, table_width / 2, 0.02]
# leg_half_extents = [leg_length / 2, leg_width / 2, leg_height / 2]
#
# # Adding friction coefficient
# table_friction = 0.5
#
# # Adjusted base position for the tabletop to be on top of the legs
# table_base_position = [0, 0, leg_height + 0.02 / 2]
#
# # Create a table using a box shape
# table_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[table_length / 2, table_width / 2, table_height / 20])
# table_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[table_length / 2, table_width / 2, table_height / 20],
#                                    rgbaColor=table_color)
# tableId = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=table_collision, baseVisualShapeIndex=table_visual,
#                             basePosition=table_base_position)
#
#
# # Function to add a leg to the table
# def add_table_leg(base_pos):
#     leg_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=leg_half_extents)
#     leg_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=leg_half_extents, rgbaColor=[0.5, 0.3, 0.2, 1])
#     p.createMultiBody(baseMass=0, baseCollisionShapeIndex=leg_collision, baseVisualShapeIndex=leg_visual,
#                       basePosition=base_pos)
#
#
# # Positions of the four legs relative to the table center
# leg_positions = [
#     [-table_length / 2 + leg_length / 2, -table_width / 2 + leg_width / 2, leg_height / 2],
#     [table_length / 2 - leg_length / 2, -table_width / 2 + leg_width / 2, leg_height / 2],
#     [-table_length / 2 + leg_length / 2, table_width / 2 - leg_width / 2, leg_height / 2],
#     [table_length / 2 - leg_length / 2, table_width / 2 - leg_width / 2, leg_height / 2],
# ]
#
# # Add legs to the table
# for position in leg_positions:
#     add_table_leg(position)
#
# # Adjust the table's dynamics, especially its restitution
# tableRestitution = 0.9  # Setting a high restitution for the table to improve bounce
# p.changeDynamics(tableId, -1, restitution=tableRestitution, lateralFriction=table_friction)
#
#
# # Function to add boundary lines as thin boxes
# def add_boundary_line(base_pos, half_extents, color=None):
#     if color is None:
#         color = [1, 1, 1, 1]
#     line_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
#     line_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
#     line_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=line_collision, baseVisualShapeIndex=line_visual,
#                                 basePosition=base_pos)
#     p.changeDynamics(line_id, -1, restitution=tableRestitution, lateralFriction=table_friction)
#
#
# # line dimensions and positions
# line_thickness = 0.02
# line_offset = 0.001
#
# # End lines
# end_line_half_extents = [line_thickness / 2, (table_width / 2) - (line_thickness / 2), line_offset]
# # Side lines
# side_line_half_extents = [(table_length / 2) - (line_thickness / 2), line_thickness / 2, line_offset]
# # Center line
# center_line_half_extents = [(table_length / 2) - (line_thickness / 2), line_thickness / 2, line_offset]
#
# # Adjusted positions to ensure lines do not extend out of the table
# add_boundary_line([table_length / 2 - line_thickness / 2, 0, table_height + 0.028 + 0.02 + line_offset],
#                   end_line_half_extents)  # Right end line
# add_boundary_line([-table_length / 2 + line_thickness / 2, 0, table_height + 0.028 + 0.02 + line_offset],
#                   end_line_half_extents)  # Left end line
# add_boundary_line([0, table_width / 2 - line_thickness / 2, table_height + 0.028 + 0.02 + line_offset],
#                   side_line_half_extents)  # Top side line
# add_boundary_line([0, -table_width / 2 + line_thickness / 2, table_height + 0.028 + 0.02 + line_offset],
#                   side_line_half_extents)  # Bottom side line
# # Center line placed correctly on the table
# add_boundary_line([0, 0, table_height + 0.028 + 0.02 + line_offset], center_line_half_extents)  # Corrected center line
#
# # ______________________________Loading in net__________________________________
# # Net dimensions
# net_height = 0.1525  # meters
# net_thickness = 0.02  # meters
# net_color = [1, 1, 1, 1]  # White color for the net
#
# net_half_extents = [net_thickness / 2, table_width / 2, net_height / 2]
#
# net_friction = 0.6
# net_base_position = [0, 0, table_height + 0.02 + net_height / 2]
#
# # Create the net
# net_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=net_half_extents)
# net_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=net_half_extents, rgbaColor=net_color)
# netId = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=net_collision, baseVisualShapeIndex=net_visual,
#                           basePosition=net_base_position)
# p.changeDynamics(netId, -1, lateralFriction=net_friction, restitution=0.1)
#
# # ______________________________Loading in ball__________________________________
# # Load a ping pong ball and set its position
# ball_mass = 0.0027
# ball_radius = 0.02
# ball_collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_radius)
# ball_visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=ball_radius, rgbaColor=[1, 1, 1, 1])
# inertia = (2 / 3) * ball_mass * (ball_radius ** 2)
#
#
# # Initialization and environment setup omitted for brevity...
# def reset_ball():
#     x_pos = -table_length / 2 + random.uniform(0.1,
#                                                table_length / 4)  # random.uniform(-table_length / 4, -table_length / 8)
#     y_pos = random.uniform(-table_width / 2,
#                            table_width / 2)  # random.uniform(-table_width / 2 + 0.1, table_width / 2 - 0.1)
#     z_pos = table_height + 0.5  # table_height + ball_radius + 0.1
#
#     ball_position = [x_pos, y_pos, z_pos]
#     ball_orientation = p.getQuaternionFromEuler([0, 0, 0])
#
#     ball_reset_id = p.createMultiBody(ball_mass, ball_collision_shape, ball_visual_shape, ball_position,
#                                       ball_orientation)
#
#     p.changeDynamics(ball_reset_id, -1, restitution=0.9, mass=ball_mass,
#                      localInertiaDiagonal=[inertia, inertia, inertia],
#                      spinningFriction=0.001, rollingFriction=0.001, lateralFriction=0.2)
#
#     return ball_reset_id
#
#
# def shoot_ball(ball_id, initial_velocity):
#     """
#     Applies an impulse to the ball to simulate hitting it. The impulse is derived
#     from the desired initial velocity.
#     """
#     impulse = np.array(initial_velocity) * ball_mass  # Impulse = velocity change * mass
#     p.applyExternalForce(ball_id, -1, impulse, [0, 0, 0], p.LINK_FRAME)
#
#
# def draw_debug_trajectory(points, color=[1, 0, 0], duration=2, width=2):
#     """
#     Draws a series of debug lines between consecutive points in a trajectory.
#     """
#     for i in range(len(points) - 1):
#         p.addUserDebugLine(points[i], points[i + 1], color, lineWidth=width, lifeTime=duration)
#
#
# def initialize_and_shoot_ball():
#     """
#     Resets the ball to a starting position and applies an initial impulse to simulate shooting it.
#     """
#     ball_id = reset_ball()  # Assuming reset_ball() positions the ball correctly
#     initial_velocity = [10, 0, 10]  # Example initial velocity; adjust as needed
#     shoot_ball(ball_id, initial_velocity)
#     return ball_id
#
#
# def initialize_kalman_filter():
#     kf = KalmanFilter(dim_x=6, dim_z=3)  # 6 state variables (x, y, z, vx, vy, vz), 3 measurements (x, y, z)
#     dt = 1.0 / 240  # Time step (assuming 240 Hz simulation frequency)
#
#     # State transition matrix (models physics: x = x0 + vx*dt, etc.)
#     kf.F = np.array([[1, 0, 0, dt, 0, 0],
#                      [0, 1, 0, 0, dt, 0],
#                      [0, 0, 1, 0, 0, dt],
#                      [0, 0, 0, 1, 0, 0],
#                      [0, 0, 0, 0, 1, 0],
#                      [0, 0, 0, 0, 0, 1]])
#
#     # Measurement matrix (we measure positions directly)
#     kf.H = np.array([[1, 0, 0, 0, 0, 0],
#                      [0, 1, 0, 0, 0, 0],
#                      [0, 0, 1, 0, 0, 0]])
#
#     # Measurement noise covariance
#     kf.R = np.eye(3) * 0.01  # Assume very low measurement noise
#
#     # Process noise covariance (adjust these values based on the expected accuracy of your model)
#     kf.Q = np.eye(6) * 0.1
#
#     # Initial state covariance
#     kf.P *= 10
#
#     return kf
#
#
# def predict_future_states(kf, steps=10):
#     # Copy the Kalman Filter to not affect its current state
#     kf_copy = deepcopy(kf)  # from copy import deepcopy
#
#     future_positions = []
#     for _ in range(steps):
#         kf_copy.predict()
#         future_pos = kf_copy.x[:3].flatten()  # Extract and flatten the position part of the state vector
#         future_positions.append(future_pos)
#
#     return future_positions
#
#
# # ______________________________Ball Logic__________________________________
# def is_ball_oob(oob_ball_id):
#     ball_pos, _ = p.getBasePositionAndOrientation(oob_ball_id)
#     return ball_pos[2] < table_height - 0.1 or abs(ball_pos[0]) > table_length / 2 or abs(ball_pos[1]) > table_width / 2
#
#
# def is_ball_stopped(stopped_ball_id):
#     threshold = 0.1
#     velocity, _ = p.getBaseVelocity(stopped_ball_id)
#     total_velocity = sum([abs(v) for v in velocity])  # Calculate the total velocity (magnitude)
#     return total_velocity < threshold
#
#
# def simulate_and_predict(ball_id):
#     """
#     Runs the simulation and uses the Kalman filter for trajectory prediction.
#     """
#     kf = initialize_kalman_filter()  # Assuming initialize_kalman_filter() is defined correctly
#     # Main simulation loop
#     for _ in range(500):  # Adjust number of steps as needed
#         p.stepSimulation()
#         time.sleep(1. / 240.)  # Simulation time step
#
#         # Update Kalman filter with current position
#         current_position, _ = p.getBasePositionAndOrientation(ball_id)
#         kf.update(np.array(current_position))
#
#         # Predict future states and draw trajectory
#         future_positions = predict_future_states(kf, steps=30)  # Adjust steps as needed
#         draw_debug_trajectory(future_positions, color=[0, 0, 1], duration=5, width=3)
#
#         # Check for ball out of bounds or stopped, then reset or continue
#         if is_ball_oob(ball_id) or is_ball_stopped(ball_id):
#             p.removeBody(ball_id)
#             ball_id = initialize_and_shoot_ball()
#
#
# # Initialization and environment setup code here...
# # Load and set up your table, ball, and environment as before
#
# ball_id = initialize_and_shoot_ball()
# shoot_ball(ball_id, [10, 0, 10])
# simulate_and_predict(ball_id)
#
# p.disconnect()
