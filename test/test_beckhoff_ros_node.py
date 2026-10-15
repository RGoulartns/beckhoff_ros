"""Test the beckhoff_ros node against a simulated PLC (ads_simulator.py on 127.0.0.1:48898)."""

import json
import os
import sys
import tempfile
import time
import unittest

from beckhoff_interfaces.msg import PlcVariable
from beckhoff_interfaces.srv import WritePlcVariable
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ads_simulator import AdsSimulator  # noqa: E402, I100

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'params.json')
INITIAL_VALUES = {
    'GVL.obj2': {'attr1': 7, 'attr2': True},
    'GVL.var1': True,
    'GVL.var2': [1, 2, 3, 4, 5],
}


@pytest.mark.launch_test
def generate_test_description():
    with open(CONFIG) as config_file:
        config = json.load(config_file)
    config['PLC']['net_id'] = '127.0.0.1.1.1'
    simulator = AdsSimulator(config['PLC'], values=INITIAL_VALUES).start()
    config_path = os.path.join(tempfile.mkdtemp(), 'params.json')
    with open(config_path, 'w') as config_file:
        json.dump(config, config_file)
    node = launch_ros.actions.Node(
        package='beckhoff_ros',
        executable='beckhoff_ros_node',
        name='beckhoff_ros_node',
        parameters=[{'config_file': config_path}],
        output='screen')
    return (
        launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]),
        {'node': node, 'simulator': simulator})


class TestBeckhoffRosNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_beckhoff_ros_node')
        cls.messages = []
        cls.node.create_subscription(
            PlcVariable, 'variable_state',
            lambda msg: cls.messages.append((msg.name, msg.type, msg.value)), 10)
        cls.client = cls.node.create_client(WritePlcVariable, 'write_variable')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, condition, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not condition():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return condition()

    def write(self, name, value_type, value):
        self.assertTrue(self.client.wait_for_service(timeout_sec=10.0))
        request = WritePlcVariable.Request()
        request.variable.name = name
        request.variable.type = value_type
        request.variable.value = value
        future = self.client.call_async(request)
        self.assertTrue(self.spin_until(future.done))
        return future.result().result

    def test_1_initial_values(self, proc_output, node):
        proc_output.assertWaitFor(
            "new message: GVL.obj2 {'attr1': 7, 'attr2': True}", process=node, timeout=10)
        proc_output.assertWaitFor('new message: GVL.var1 True', process=node, timeout=10)

    def test_2_plc_changes_are_published(self, simulator):
        self.assertTrue(self.spin_until(lambda: self.node.count_publishers('variable_state')))
        self.spin_until(lambda: False, timeout=1.0)  # let the publisher match the subscription
        simulator.set_value('GVL.obj2', {'attr1': 9, 'attr2': False})
        expected = ('GVL.obj2', "<class 'dict'>", "{'attr1': 9, 'attr2': False}")
        self.assertTrue(self.spin_until(lambda: expected in self.messages), self.messages)

    def test_3_write_variable(self, simulator):
        self.assertTrue(self.write('GVL.var1', 'bool', '0'))
        self.assertFalse(simulator.get_value('GVL.var1'))
        expected = ('GVL.var1', "<class 'bool'>", 'False')
        self.assertTrue(self.spin_until(lambda: expected in self.messages), self.messages)

    def test_4_write_structures(self, simulator):
        value = [{'attr1': 1, 'attr2': {'attr1': 2, 'attr2': True}, 'attr3': [True, False]}]
        self.assertTrue(self.write('GVL.obj1', 'list', repr(value)))
        zero = {'attr1': 0, 'attr2': {'attr1': 0, 'attr2': False}, 'attr3': [False, False]}
        self.assertEqual(simulator.get_value('GVL.obj1'), value + [zero] * 9)

    def test_5_invalid_writes(self, simulator):
        writes = len(simulator.writes)
        # not in variables_to_write
        self.assertFalse(self.write('GVL.obj2', 'dict', "{'attr1': 1, 'attr2': True}"))
        self.assertFalse(self.write('GVL.unknown', 'int', '1'))
        # invalid values
        self.assertFalse(self.write('GVL.var1', 'bool', 'maybe'))
        self.assertFalse(self.write('GVL.var1', 'str', 'True'))
        self.assertFalse(self.write('GVL.obj1', 'list', "[{'attr1': 1}]"))
        self.assertEqual(len(simulator.writes), writes)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info, simulator):
        simulator.stop()
        launch_testing.asserts.assertExitCodes(proc_info)
