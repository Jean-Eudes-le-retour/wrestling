# Copyright 1996-2023 Cyberbotics Ltd.
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

class Average():
    """Class that provides an interface to the average of a list of values."""

    def __init__(self, dimensions, historySteps=10):
        self.HISTORY_STEPS = historySteps
        if dimensions > 1:
            self.isVector = True
            self.average = [0]*dimensions
            self.history = [[0]*dimensions]*self.HISTORY_STEPS
        else:
            self.isVector = False
            self.average = 0
            self.history = [0]*self.HISTORY_STEPS

    def get_new_average(self, value):
        """Returns the current accelerometer average of the last HISTORY_STEPS values."""
        self.updateAverage(value)
        return self.average

    def updateAverage(self, value):
        """Updates the average with a new value."""
        self.history.pop(0)
        self.history.append(value)
        if self.isVector:
            self.average = [sum(col)/self.HISTORY_STEPS for col in zip(*self.history)]
        else:
            self.average = sum(self.history)/self.HISTORY_STEPS
