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
    "GET_BOARD_INFO": 1, "UPDATE_PWM_VALUES": 2,
    "UPDATE_PID_SETPOINTS": 3,
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


class Output():
    def __init__(self, type: str, channel: float, pin: float, control_mode: str = "MANUAL"):
        validate_output_type(type)
        validate_control_mode(control_mode)
        self.type = type
        self.control_mode = control_mode
        self.channel = channel
        self.pin = pin

    def __repr__(self):
        return f"Output(type={self.type}, channel={self.channel}, pin={self.pin}, control_mode={self.control_mode})"


class Input():
    def __init__(self, type: str, channel: float, variable: str = None):
        validate_input_type(type)
        self.type = type
        self.channel = channel
        self.variable = variable

    def __repr__(self):
        return f"Analog_input(type={self.type}, channel={self.channel}, variable={self.variable})"


class Board():
    def __init__(self, ADDRESS: str, port_name: str = "/dev/ttyUSB0", baud_rate: float = 230400):
        self.ADDDRESS = ADDRESS
        self.set_serial_port(port_name)
        self.set_baud_rate(baud_rate)
        self.open_connection()
        print("Connection successfull\n")
        self.set_outputs(board_info)
        self.set_inputs(board_info)

    def set_serial_port(self, port_name):
        validate_serial_port(port_name)
        self.port_name = port_name

    def set_baud_rate(self, baud_rate):
        validate_baud_rate(baud_rate)
        self.baud_rate = baud_rate

    def open_connection(self):
        self.serial_port = serial.Serial(port=self.port_name, baudrate=self.baud_rate, timeout=2)

    def set_outputs(self, board_info):
        self.outputs_list = []
        for type, channel, pin, control_mode in board_info["outputs"]:
            validate_output_type(type)
            self.outputs_list.extend([Output(type, channel, pin, control_mode)])

        print(f"{len(self.outputs_list)} output channels detected:")
        for i in self.outputs_list:
            print(i)

    def set_inputs(self, board_info):
        self.inputs_list = []
        for type, channel, variable in board_info["inputs"]:
            validate_input_type(type)
            self.inputs_list.extend([Input(type, channel, variable)])

        print(f"{len(self.inputs_list)} input channels detected:")
        for i in self.inputs_list:
            print(i)

    def read_data(self):
        data_str = self.readline()
        if self.is_valid_input_string(data_str):
            data_dict = self.load_json_to_dict(data_str)
            return data_dict
        else:
            return {}

    def readline(self):
        return self.serial_port.readline().decode("UTF-8")

    def is_valid_input_string(self, input_string: str) -> bool:
        if "115,!" in input_string:
            return True
        else:
            return False

    def load_json_to_dict(self, json_string: str) -> dict:
        valid_json_string = self.convert_input_string_to_json(json_string)
        return ast.literal_eval(valid_json_string)

    def convert_input_string_to_json(self, input_string: str) -> str:
        return input_string.replace("115,!", "").replace("'", '"').replace(", }", "}")


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
