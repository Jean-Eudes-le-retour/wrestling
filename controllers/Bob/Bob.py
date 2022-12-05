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

"""Minimalist controller example for the Robot Wrestling Tournament.
   Beats Alice by moving forwards and therefore having a higher coverage."""

from controller import Robot, Motion


class Wrestler (Robot):
    def run(self):
        forward = Motion('../motions/Forwards50.motion')
        forward.play()
        timeStep = int(self.getBasicTimeStep())  # retrieves the WorldInfo.basicTimeTime (ms) from the world file
        while self.step(timeStep) != -1:
            pass


# create the Robot instance and run main loop
wrestler = Wrestler()
wrestler.run()
