#!python3.8

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from .beckhoff_plc import BeckhoffPLC
from beckhoff_interfaces.msg import PlcVariable
from beckhoff_interfaces.srv import WritePlcVariable

import os
import json
from pydoc import locate


class BeckhoffROS(Node):
    def __init__(self):
        super().__init__('beckhoff_ros')

        # ALTERNATIVE METHOD: LOAD JSON PARAMETERS
        file = os.path.join('INSERT_PATH/src/beckhoff_ros','config','params.json')
        with open(file, "r") as content:
            jsonData = json.load(content)
        
        # ROS2 Parameters
        params = {}
        self.declare_parameters(
            namespace='',
            parameters=[
                ('net_id', '192.168.1.10.1.1'),
                ('client_net_id', '192.168.1.20.1.1'),
                ('username', 'Administrator'),
                ('password', '1'),
                ('variables', None),
                ('variables_to_monitor', None),
                ('variables_to_write', None)
            ]
        )
        
        params['net_id'] = self.get_parameter('net_id').value
        params['client_net_id'] = self.get_parameter('client_net_id').value
        params['username'] = self.get_parameter('username').value
        params['password'] = self.get_parameter('password').value
        # Yaml cannot 'easily' read dictionary of dictionaries 
        params['variables'] = self.get_parameter('variables').value
        params['variables_to_monitor'] = self.get_parameter('variables_to_monitor').value
        params['variables_to_write'] = self.get_parameter('variables_to_write').value

        # self.plc = BeckhoffPLC(params)
        self.plc = BeckhoffPLC(jsonData["PLC"])
        self.plc.addObserver(self.onPlcChanged)

        cbGroup = ReentrantCallbackGroup()
        self.writeVar_srv = self.create_service(WritePlcVariable, 'write_variable', self.WriteVarSrvCB, callback_group=cbGroup)
        self.varChanged_pub = self.create_publisher(PlcVariable, 'variable_state', 1)
        
        


    def onPlcChanged(self, varName: str, varContents):
        print(f"new message: {varName} {varContents}")
        msg = PlcVariable()
        msg.name = varName
        msg.type = str(type(varContents))
        msg.value = str(varContents)
        self.varChanged_pub.publish(msg)


    def WriteVarSrvCB(self, request, response):
        varType = locate(request.variable.type)
        
        # workaround to cast to bool. Boolean value must be 0 or 1.
        if varType is bool:
            varValue = bool(int(request.variable.value))
        else:
            varValue = varType(request.variable.value)

        response.result = self.plc.setVariable(request.variable.name, varValue)
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        beckhoffROS = BeckhoffROS()

        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(beckhoffROS)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            beckhoffROS.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
