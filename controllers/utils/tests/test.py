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

import ik
ik.np.set_printoptions(suppress=True)
PI_4 = ik.np.pi/4
print("PI_4", PI_4)
# from [0, 50, -300]:
_, left = ik.fLeftLeg([0., 0., -0.58785418, 1.15704096, -0.56918678, 0.0])
print(left)

import old_ik
old_ik.np.set_printoptions(suppress=True)
# [0., 50., -209.77999986, 3.14159265, 0., -3.14159265]
result = old_ik.inverse_left_leg(old_ik.makeTransformation(left[0], left[1], left[2], left[3], left[4], left[5]))
print(result)