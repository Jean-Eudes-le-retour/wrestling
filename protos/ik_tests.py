import ikpy
from ikpy.chain import Chain
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

left_leg_chain = Chain.from_urdf_file(
    'nao.urdf',
    base_elements=['base_link', 'LHipYawPitch'],
    active_links_mask=[False, True, True, True, True, True, True, False]
)

right_leg_chain = Chain.from_urdf_file(
    'nao.urdf',
    base_elements=['base_link', 'RHipYawPitch'],
    active_links_mask=[False, True, True, True, True, True, True, False]
)

# links=['Base link', 'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll', 'LLeg_effector_fixedjoint']
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

start_joint = [0, 0, 0, -0.523, 1.047, -0.524, 0, 0]

left_ik = left_leg_chain.inverse_kinematics([0, 0.05, -0.3], [0, 0, 0], initial_position=start_joint, orientation_mode='all')
print(left_ik)

left_leg_chain.plot(left_ik, ax)
right_leg_chain.plot(right_leg_chain.inverse_kinematics([0, -0.05, -0.3], [0, 0, 0], initial_position=start_joint, orientation_mode='all'), ax)

ax.set_box_aspect([ub - lb for lb, ub in (getattr(ax, f'get_{a}lim')() for a in 'xyz')])
plt.show()
