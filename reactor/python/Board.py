#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 2024 Feb 29 12:00

@author: Alexis Saldivar
"""

import ast

from serial.tools import list_ports
from serial.serialutil import SerialException

import serial

valid_baud_rates = [230400]
valid_commands = {
    "GET_BOARD_INFO": 0, "TOGGLE_CONTROL_MODE": 1,
}
valid_control_modes = {"MANUAL": 0, "TIMER": 1, "PID": 2, "ONOFF": 3}
valid_input_types = ["analog", "i2c", "spi", "flow"]
valid_output_types = ["pwm", "digital"]
board_info = {"outputs": [("pwm", 0, 13, "MANUAL"), ("pwm", 1, 12, "MANUAL"),
                          ("pwm", 2, 14, "MANUAL"), ("pwm", 3, 27, "MANUAL"),
                          ("pwm", 4, 26, "MANUAL"), ("pwm", 5, 25, "MANUAL")],
              "inputs": [("analog", 0, "current"), ("analog", 1, "dissolved_oxygen"),
                         ("analog", 2, "ph"), ("analog", 3, "temperature_0"),
                         ("analog", 4, "temperature_1"), ("analog", 5, "temperature_2"),
                         ("analog", 6, "temperature_3"), ("analog", 7, "temperature_4")],
              }


class Dictlist(list):
    def __init__(self, iterable: list) -> None:
        list.extend(self, iterable)
        self._dict = self._generate_index()

    def _generate_index(self) -> dict:
        return {obj.id: k for k, obj in enumerate(self)}

    def get_by_id(self, id: str):
        return list.__getitem__(self, self._dict[id])


class Output():
    def __init__(self, type: str, channel: float, pin: float, control_mode: str = "MANUAL"):
        validate_output_type(type)
        validate_control_mode(control_mode)
        self.id = type + "_" + str(channel)
        self.type = type
        self.control_mode = control_mode
        self.channel = channel
        self.pin = pin

    def __repr__(self):
        return f"Output(type={self.type}, channel={self.channel}, pin={self.pin}, control_mode={self.control_mode})"


class Input():
    def __init__(self, type: str, channel: float, id: str):
        validate_input_type(type)
        self.id = id
        self.type = type
        self.channel = channel

    def __repr__(self):
        return f"Input(type={self.type}, channel={self.channel}, id={self.id})"


class Board():
    def __init__(self, address: str, port_name: str = "/dev/ttyUSB0", baud_rate: float = 230400):
        self.address = address
        self.set_serial_port(port_name)
        self.set_baud_rate(baud_rate)
        self.open_connection()
        print("Connection successfull\n")
        board_info_dict = self.request_board_info()
        self.set_outputs(board_info_dict)
        self.set_inputs(board_info_dict)

    def set_serial_port(self, port_name):
        validate_serial_port(port_name)
        self.port_name = port_name

    def set_baud_rate(self, baud_rate):
        validate_baud_rate(baud_rate)
        self.baud_rate = baud_rate

    def open_connection(self):
        self.serial_port = serial.Serial(port=self.port_name, baudrate=self.baud_rate, timeout=2)

    def request_board_info(self):
        board_info_str = self.write("GET_BOARD_INFO")
        return self.load_json_to_dict(board_info_str)

    def set_outputs(self, board_info):
        outputs_list = []
        for type, channel, pin, mode in board_info["outputs"]:
            validate_output_type(type)
            control_mode = list(valid_control_modes.keys())[mode]
            outputs_list.extend([Output(type, channel, pin, control_mode)])
        self.Outputs = Dictlist(outputs_list)

        print(f"{len(self.Outputs)} output channels detected:")
        for i in self.Outputs:
            print(i)

    def set_inputs(self, board_info):
        inputs_list = []
        for type, channel, variable in board_info["inputs"]:
            validate_input_type(type)
            inputs_list.extend([Input(type, channel, variable)])
        self.Inputs = Dictlist(inputs_list)

        print(f"{len(self.Inputs)} input channels detected:")
        for i in self.Inputs:
            print(i)

    def read_data(self):
        data_str = self.readline()
        if self.is_valid_data_string(data_str):
            data_dict = self.load_json_to_dict(data_str)
            return data_dict
        else:
            return {}

    def readline(self):
        return self.serial_port.readline().decode("UTF-8")

    def is_valid_data_string(self, input_string: str) -> bool:
        conditions = [input_string[0] == "{", "100,!" in input_string]
        return all(conditions)

    def load_json_to_dict(self, json_string: str) -> dict:
        valid_json_string = self.convert_input_string_to_json(json_string)
        return ast.literal_eval(valid_json_string)

    def convert_input_string_to_json(self, input_string: str) -> str:
        return input_string.replace("115,!", "").replace("'", '"').replace(", }", "}").replace("100,!", "")

    def write(self, command: str, args: list = None):
        confirmation = False
        cmd_str = self.build_cmd_str(command, args)
        while not confirmation:
            self.serial_port.write(cmd_str.encode())
            self.serial_port.flush()
            if self.serial_port.inWaiting() > 0:
                input = self.readline()
                if self.is_valid_input_string(input):
                    confirmation = True

        return input

    def is_valid_input_string(self, input_string: str) -> bool:
        conditions = [input_string[0] == "{", "115,!" in input_string]
        return all(conditions)

    def build_cmd_str(self, command: str, args: list = None):
        """
        Build a command string that can be sent to the arduino.

        Input:
            command (str): the command to send to the arduino, must not
                contain a % character
            args (iterable): the arguments to send to the command
        """
        try:
            cmd = valid_commands[command]
        except KeyError:
            print(f"{command} not a valid command, must be one of {valid_commands}")

        if args:
            args = ','.join(map(str, args))
        else:
            args = ''
        return self.address + " {cmd},{args}!\n".format(cmd=cmd, args=args)


def validate_serial_port(port_name):
    available_ports = get_available_serial_ports()
    if port_name not in available_ports:
        raise ValueError(f"{port_name} not available. Did you mean one of {available_ports}")

    return True


def get_available_serial_ports() -> list:
    serial_ports = list_ports.comports()
    return [port.device for port in serial_ports]


def validate_baud_rate(baud_rate: str) -> bool:
    if baud_rate not in valid_baud_rates:
        raise ValueError(f"Baud rate {baud_rate} not valid. Must be one of {valid_baud_rates[:]}")

    return True


def validate_input_type(type: str) -> bool:
    if type not in valid_input_types:
        raise ValueError(f"type must be one of {valid_input_types}")

    return True


def validate_output_type(type: str) -> bool:
    if type not in valid_output_types:
        raise ValueError(f"type must be one of {valid_output_types}")

    return True


def validate_control_mode(mode: str) -> bool:
    if mode not in valid_control_modes:
        raise ValueError(f"control_mode must be one of {valid_control_modes}")
