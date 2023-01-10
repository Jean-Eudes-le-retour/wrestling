# Copyright 1996-2022 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
from scipy.spatial.transform import Rotation as R
import ik
import numpy as np
sys.path.append('..')
from kinematics import Kinematics
ik.np.set_printoptions(suppress=True)
PI_4 = ik.np.pi/4
PI = ik.np.pi


import time

# about x100 faster
start_time = time.time()
print(ik.inverse_leg(0, 50, -300, 0, 0, 0, is_left=True))
print("--- %s seconds ---" % (time.time() - start_time))
print(ik.forward_left_leg([0., 0., -0.5878541756344366, 1.1570409574741018, -0.5691867818396654, 0.]))
print('------------------')

start_time = time.time()
ik_class = Kinematics()
print(ik_class.ik_left_leg([0, 0.05, -0.3]))
print("--- %s seconds ---" % (time.time() - start_time))
print('------------------')
print('------------------')
start_time = time.time()
print(ik.inverse_leg(0, -50, -300, 0, 0, 0, is_left=False))
print("--- %s seconds ---" % (time.time() - start_time))
print(ik.forward_right_leg([0., 0., -0.5878541756344366, 1.1570409574741018, -0.5691867818396654, 0.]))
print('------------------')

start_time = time.time()
ik_class = Kinematics()
print(ik_class.ik_right_leg([0, -0.05, -0.3]))
print("--- %s seconds ---" % (time.time() - start_time))

