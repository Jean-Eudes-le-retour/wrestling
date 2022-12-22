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

class Accelerometer():
    """Class that provides an interface to the accelerometer sensor."""

    def __init__(self, device, timeStep, historySteps=10):
        self.device = device
        self.device.enable(timeStep)
        self.average = [0]*3
        self.HISTORY_STEPS = historySteps
        self.history = [[0]*3]*self.HISTORY_STEPS

    def getValues(self):
        """Returns the current accelerometer values."""
        return self.device.getValues()

    def getAverage(self):
        """Returns the current accelerometer average of the last HISTORY_STEPS values."""
        return self.average

    def _updateAverage(self, values):
        """Updates the accelerometer average."""
        self.history.pop(0)
        self.history.append(values)
        self.average = [sum(col)/self.HISTORY_STEPS for col in zip(*self.history)]

    def update(self):
        """Updates the accelerometer average and returns the current accelerometer values."""
        values = self.getValues()
        self._updateAverage(values)
        return values