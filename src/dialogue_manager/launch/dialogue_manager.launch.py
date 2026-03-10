# Copyright (c) 2026 TODO. All rights reserved.
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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_pal import get_pal_configuration


def generate_launch_description():
    pkg = 'dialogue_manager'
    node = 'dialogue_manager'
    ld = LaunchDescription()

    # automatically fetch the start parameters for this node,
    # using defaults installed with the skill, as well as possible
    # user overrides
    config = get_pal_configuration(pkg=pkg, node=node, ld=ld)

    node = Node(
        package=pkg,
        executable='dialogue_manager_node',
        namespace='',
        name=node,
        parameters=config['parameters'],
        remappings=config['remappings'],
        arguments=config['arguments'],
        output='both', emulate_tty=True,
    )

    ld.add_action(node)

    return ld
