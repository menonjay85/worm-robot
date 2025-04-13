from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.articulations import ArticulationView

prim_path = "/new_assem_wo_mating"
prim = get_prim_at_path(prim_path)

if prim is None:
    raise ValueError(f"Prim at path {prim_path} does not exist.")
else:
    print(f"Found prim at path: {prim_path}")

# Initialize the robot articulation
robot_prim_path = "/new_assem_wo_mating"
robot = Articulation(prim_path=robot_prim_path, name="5DOF_Robot")

# Initialize the articulation
robot.initialize()

robot.set_solver_position_iteration_count(32)
robot.set_enabled_self_collisions(True)

# Access basic properties
print("Number of DOFs:", robot.num_dof)
print("DOF Names:", robot.dof_names)
print("DOF Properties:", robot.dof_properties)
print("Number of Bodies:", robot.num_bodies)
print("Efforts:", robot.dof_properties["maxEffort"])
print("Efforts:", robot.dof_properties["lower"])
print("Efforts:", robot.dof_properties["upper"])
print("Efforts:", robot.dof_properties["stiffness"])

print(robot.get_dof_index("joint_1"))
#robot.disable_gravity()
robot.enable_gravity()

# Set joint positions directly (teleports the joints)
robot.set_joint_positions([0.0, -0.1, 0.0, 0.1, -0.1])
print(robot.get_joint_positions())

robot.set_joint_efforts(np.array([0.0, 0.0, 0.0, 0.0, 0.0]))
print(robot.get_applied_joint_efforts())
print(robot.get_measured_joint_forces())

metadata = robot._articulation_view._metadata
joint_indices = 1 + np.array([metadata.joint_indices["joint_4"], metadata.joint_indices["joint_5"],])
print(joint_indices)
print(robot.get_measured_joint_forces(joint_indices))

state = robot.get_joints_default_state()
print(state)
print(state.positions)
print(state.velocities)

robot.set_joints_default_state(positions=np.array([0.3, 0.0, 0.0, 0.0, 0.0]), velocities=np.zeros(shape=(robot.num_dof,)), efforts=np.zeros(shape=(robot.num_dof,)))
robot.post_reset()

print(robot.get_articulation_controller())

robot.set_linear_velocity(np.array([0.0, 0.0, 0.0]))

robot.set_angular_velocity(np.array([0, 0, 0]))

action = ArticulationAction(joint_positions=np.array([0.0, -1.0, 0.0, -2.2, 0.0]))
robot.apply_action(action)

# close the robot fingers: panda_finger_joint1 (7) and panda_finger_joint2 (8) to 0.0
action = ArticulationAction(joint_positions=np.array([0.5, 0.0]), joint_indices=np.array([3, 4]))
robot.apply_action(action)

print(robot.get_applied_action())
print(robot.get_solver_position_iteration_count())


robot.set_sleep_threshold(5)
print(robot.get_sleep_threshold())

robot = ArticulationView(prim_paths_expr=robot_prim_path, name="5DOF_Robot_av")

robot.initialize()
print(robot.initialized)
print(robot)
print(robot.num_dof)

positions, orientations = robot.get_body_coms()
print(positions)
print(orientations)

print(robot.get_body_masses())
print(robot.get_coriolis_and_centrifugal_forces())

stiffnesses, dampings = robot.get_gains()
print(stiffnesses)
print(dampings)

stiffnesses = np.tile(np.array([70000, 50000]), (1, 1))
dampings = np.tile(np.array([2000, 2000]), (1, 1))
robot.set_gains(kps=stiffnesses, kds=dampings, joint_indices=np.array([3, 4]))

stiffnesses, dampings = robot.get_gains()
print("New:")
print(stiffnesses)
print(dampings)