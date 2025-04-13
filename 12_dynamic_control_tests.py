from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.dynamic_control import conversions as dc_conversions
from pxr import Gf

dc = _dynamic_control.acquire_dynamic_control_interface()

art = dc.get_articulation("/new_assem_wo_mating/base_link")

num_joints = dc.get_articulation_joint_count(art)
num_dofs = dc.get_articulation_dof_count(art)
num_bodies = dc.get_articulation_body_count(art)

print(num_joints)
print(num_dofs)
print(num_bodies)
dof_ptr = dc.find_articulation_dof(art, "joint_2")

dof_type = dc.get_dof_type(dof_ptr)
# print position for the degree of freedom
print(dof_type)


fixed_joint_ptr = dc.find_articulation_joint(art, "joint_4")
joint_type = dc.get_joint_type(fixed_joint_ptr)
print(joint_type)


root_body = dc.get_articulation_root_body(art)
hand_idx = dc.find_articulation_body_index(art, "joint_5")

dof_states = dc.get_articulation_dof_states(art, _dynamic_control.STATE_POS)
targets = dc.get_articulation_dof_position_targets(art)
dof_states["pos"] = targets
dc.set_articulation_dof_states(art, dof_states, _dynamic_control.STATE_POS)
body_states = dc.get_articulation_body_states(art, _dynamic_control.STATE_POS)

expected_pos = body_states["pose"]["p"][hand_idx]
new_pose = dc_conversions.create_transform(Gf.Vec3d(0.10, 0.10, 0.10), Gf.Rotation(Gf.Vec3d(0, 0, 1), 90))

dc.set_rigid_body_pose(root_body, new_pose)
body_states = dc.get_articulation_body_states(art, _dynamic_control.STATE_POS)


dof_masses = dc.get_articulation_dof_masses(art)
dof_props = dc.get_articulation_dof_properties(art)
num_dofs = dc.get_articulation_dof_count(art)