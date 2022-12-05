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

import subprocess
import os


def _test_controller(controller_name):
    try:
        with open(os.path.join(controller_name, f'{controller_name}.py'), 'r'):
            pass
    except IOError:
        subprocess.run(
            [
                'svn', 'export',
                f'https://github.com/Jean-Eudes-le-retour/{controller_name}-wrestling-controller/trunk/controllers',
                '.'
            ]
        )


# If the example controllers files are not present, downloads the example controller from the github repository:
_test_controller('Alice')
_test_controller('Bob')
_test_controller('Charlie')
_test_controller('David')
