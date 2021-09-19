#!python3.8

from typing import Callable
from functools import partial
import pyads

import ctypes
from ctypes import sizeof
from ctypes import c_ubyte
from ctypes import Structure

import traceback
import sys
sys.tracebacklimit = 0

# TODO
#  - call createroute when needed automatically
#  - handle plc not found


class BeckhoffPLC:
    cTypes = {
        'bool': ctypes.c_bool,
        'int': ctypes.c_int16,
        'int16': ctypes.c_int16,
        'uint16': ctypes.c_uint16,
        'uint8': ctypes.c_uint8,
        'real': ctypes.c_float,
        # insert type
    }

    def __init__(self, params: dict):
        self.params = params
        self.observers = []
        self.numManips = 4
        self.pack = 0

        self.connection = pyads.Connection(
            self.params["net_id"], pyads.PORT_TC3PLC1)
        self.connection.open()

        self.variables = self.loadVariables(params["variables"])

        for var in params["variables_to_monitor"]:
            self.connection.add_device_notification(
                var,
                pyads.NotificationAttrib(sizeof(self.variables[var])),
                partial(self.updateCB, self.variables[var])
            )

        self.varsHandle = {}
        for var in params["variables_to_write"]:
            self.varsHandle[var] = self.connection.get_handle(var)


    def __del__(self):
        print("plc destructor called")
        # broken: self.connection.close()


    # very simple observer pattern wannabe
    def addObserver(self, obsUpdateMethod: Callable[[str, dict], None]):
        self.observers.append(obsUpdateMethod)


    def notify(self, source: str, msg):
        for obs in self.observers:
            obs(source, msg)


    def updateCB(self, msgCType, notification, name):
        # msgCType: defines the received message (e.g. ctype Structure)
        # name: name of the variable received
        # parser: handle, timestamp, value
        if self.observers:
            _, _, value = self.connection.parse_notification(
                notification, msgCType)

            if(isinstance(value, Structure)):
                value = self.cStruct2Dict(value)
            
            self.notify(name, value)


    def setVariable(self, name, value):
        #e.g. false, 'qwer', (69, false, 1), (dict, dict), dict
        # value can be a int/str/bool or list(for arrays) dicts(for structs)
        try:
            if isinstance(value, dict):
                valueCtype = BeckhoffPLC.cTypes[name](*tuple(value.values()))
            elif isinstance(value, tuple):
                instances = list()
                for v in value:
                    instances.append(
                        self.variables[name]._type_(*list(v.values())))
                    valueCtype = self.variables[name](*instances)
            else:
                valueCtype = self.variables[name](value)

            self.connection.write_by_name(
                name,
                bytearray(valueCtype),
                c_ubyte * len(bytearray(valueCtype)),
                handle=self.varsHandle[name]
            )
            return "Done"
        except Exception:
            print(traceback.format_exc())
            return traceback.format_exc()


    def loadVariables(self, dictionary: dict):
        variables = {}
        # converts all types defined in the dictionary to ctypes
        for varName, varType in dictionary.items():
            if isinstance(varType, dict):
                # dict to ctype structure.
                # Structures are added to the "BeckhoffPLC.cTypes" for future reference
                variables[varName] = self.createCTypeStruct(
                    varName, self.pack, varType)
                BeckhoffPLC.cTypes[varName] = variables[varName]
            elif isinstance(varType, list):
                # list [type, size] to ctype
                variables[varName] = BeckhoffPLC.cTypes[varType[0]] * varType[1]
            else:
                # str to ctype
                variables[varName] = BeckhoffPLC.cTypes[varType]
        return variables


    def createCTypeStruct(self, name, pack, fields: dict):
        for fName, fType in fields.items():
            if isinstance(fType, str):
                # convert str to ctype
                fields[fName] = BeckhoffPLC.cTypes[fType]
            elif isinstance(fType, list):
                # convert array defined by [type, size] to ctype
                fields[fName] = BeckhoffPLC.cTypes[fType[0]] * fType[1]
            else:
                raise TypeError("Unexpected fields format. Fields must be either str or a list")

        cTypeClass = type(
            name,
            (Structure, ),
            {"_pack_": pack, "_fields_": list(fields.items())}
        )
        return cTypeClass


    # source: https://stackoverflow.com/questions/3789372/python-can-we-convert-a-ctypes-structure-to-a-dictionary
    def cStruct2Dict(self, struct: Structure):
        result = {}
        for field, _ in struct._fields_:
            value = getattr(struct, field)
            if hasattr(value, "_length_") and hasattr(value, "_type_"):  # Probably an array
                value = list(value)
                if hasattr(value[0], "_fields_"):
                    for i in range(len(value)):
                        value[i] = self.cStruct2Dict(
                            value[i])  # ye.. recursion.. I know
            elif hasattr(value, "_fields_"):
                # ye.. recursion.. I know .. again :P
                value = self.cStruct2Dict(value)

            result[field] = value

        return result


    # linux OS only
    def createRoute(self):
        pyads.open_port()
        pyads.set_local_address(self.params["client_net_id"])
        pyads.add_route_to_plc(
            self.params["client_net_id"],
            self.netId2ip(self.params["client_net_id"]),
            self.netId2ip(self.params["net_id"]),
            self.params["username"],
            self.params["password"],
            route_name="jetson-to-plc"
        )

    def netId2ip(self, netId: str):
        ip = netId
        for i in range(2):
            ip = ip[0:ip.rfind('.')]
        return ip
