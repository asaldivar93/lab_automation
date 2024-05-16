#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 2024 Feb 29 12:00

@author: Alexis Saldivar
"""
from typing import Callable, NamedTuple

import ast
import json
import logging
import os
import time

from serial.tools import list_ports
from serial.serialutil import SerialException
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

import serial

logger = logging.getLogger(__name__)
logger.addHandler(logging.StreamHandler())
logging.basicConfig(
    filename="record.log", encoding='utf-8',
    level=logging.INFO, format='%(asctime)s %(levelname)s: %(message)s'
)

valid_baud_rates = [230400, 115200, 74880, 57600, 38400, 19200, 9600]
valid_commands = {
    "GET_BOARD_INFO": 0, "UPDATE_CONFIGURATION": 1, "GET_ALL_DATA": 2
}
valid_control_modes = {"MANUAL": 0, "TIMER": 1, "PID": 2, "ONOFF": 3}
valid_input_types = ["adc", "i2c", "spi", "vol"]
valid_output_types = {"pwm": range(0, 255), "digital": [0, 1], "stp": range(0, 1000)}


class Dictlist(list):
    def __init__(self, iterable: list) -> None:
        list.extend(self, iterable)
        self._dict = self._generate_index()
        self._dict1 = self._generate_index1()

    def _generate_index(self) -> dict:
        return {obj.id: k for k, obj in enumerate(self)}

    def _generate_index1(self) -> dict:
        return {"_".join([obj.address, str(obj.channel)]): k for k, obj in enumerate(self)}

    def get_by_id(self, id: str):
        return list.__getitem__(self, self._dict[id])

    def get_by_channel(self, channel: str):
        return list.__getitem__(self, self._dict1[channel])


class config_handler(FileSystemEventHandler):
    def __init__(self, fun_callback: Callable):
        self.fun_callback = fun_callback

    def on_modified(self, event):
        self.fun_callback()


class Output():
    def __init__(self, address: str, type: str, channel: float):
        self.address = address
        self.id = "_".join([address, str(channel)])
        self.type = type
        self.channel = channel
        self.bounds = valid_output_types[type]
        self.control_mode = manual_control(0, 0)

    def __repr__(self):
        return f"Output(id={self.id}, channel={self.channel}, type={self.type})"


class Input():
    def __init__(self, address: str, type: str, channel: float, variable: str):
        validate_input_type(type)
        self.address = address
        self.id = "_".join([address, variable])
        self.variable = variable
        self.type = type
        self.channel = channel

    def __repr__(self):
        return f"Input(type={self.type}, channel={self.channel}, id={self.id})"


class Board():
    def __init__(self, address: str, port_name: str = "/dev/ttyUSB0", baud_rate: float = 230400, config_dir: str = "configuration/"):
        self.address = address
        self.id = address
        self.channel = 0
        self.port_name = port_name

        self.set_serial_port(port_name)
        self.set_baud_rate(baud_rate)
        self.open_connection(port_name)

        board_info_dict = self.request_board_info()
        self.samples_per_second = board_info_dict["samples_per_second"]

        self.set_outputs(board_info_dict)
        self.set_inputs(board_info_dict)
        print("Connection successfull\n")

        self.config_dir = config_dir
        self.read_config_json()

    def set_serial_port(self, port_name):
        validate_serial_port(port_name)
        self.port_name = port_name

    def set_baud_rate(self, baud_rate):
        validate_baud_rate(baud_rate)
        self.baud_rate = baud_rate

    def open_connection(self, port_name):
        self.port_name = port_name
        self.serial_port = serial.Serial(port=self.port_name, baudrate=self.baud_rate, timeout=2)

    def request_board_info(self):
        board_info_str = self.write_command(self.address, "GET_BOARD_INFO")
        return self.load_json_to_dict(board_info_str)

    def set_outputs(self, board_info):
        outputs_list = []
        address_list = board_info["outs"].keys()
        for address in address_list:
            channels_list = board_info["outs"][address]
            for type, channel in channels_list:
                outputs_list.extend([Output(address, type, channel)])
        self.Outputs = Dictlist(outputs_list)

        print(f"\n{len(self.Outputs)} output channels detected:")
        for i in self.Outputs:
            print(i)

    def set_inputs(self, board_info):
        inputs_list = []
        address_list = board_info["ins"].keys()
        for address in address_list:
            channels_list = board_info["ins"][address]
            for type, channel, variable in channels_list:
                inputs_list.extend([Input(address, type, channel, variable)])
        self.Inputs = Dictlist(inputs_list)

        print(f"\n{len(self.Inputs)} input channels detected:")
        for i in self.Inputs:
            print(i)

    def read_config_json(self):
        with open(os.path.join(self.config_dir, "config.json"), "r") as file:
            try:
                self.config_dict = json.load(file)
                self.is_config_updated = True
            except ValueError as error:
                logger.warning(f" JSONDecodeError: {error}")

    def read_data(self):
        data_str = self.write_command(self.address, "GET_ALL_DATA")
        data_dict = self.load_json_to_dict(data_str)
        return data_dict

    def readline(self):
        return self.serial_port.readline().decode("UTF-8")

    def load_json_to_dict(self, json_string: str) -> dict:
        valid_json_string = self.convert_input_string_to_json(json_string)
        try:
            return ast.literal_eval(valid_json_string)
        except SyntaxError:
            return {}
        except ValueError as e:
            logger.warning(e)
            logger.warning(valid_json_string)
            return {}

    def convert_input_string_to_json(self, input_string: str) -> str:
        return input_string.replace("115,!", "").replace("'", '"').replace(", }", "}")

    def update_configuration(self):
        if self.is_config_updated:
            command = "UPDATE_CONFIGURATION"
            args_queue = self.parse_config(self.config_dict)
            while args_queue:
                print(args_queue[0])
                address, args = args_queue[0]
                self.write_command(address, command, args)
                args_queue.pop(0)
            self.is_config_updated = False
            print("Board configuration updated")

    def write_command(self, address: str, command: str, args: list = None):
        confirmation = False
        cmd_str = self.build_cmd_str(address, command, args)
        self.serial_port.flushInput()
        self.serial_port.write(cmd_str.encode())
        while not confirmation:
            input = self.readline()
            if self.is_valid_input_string(input):
                confirmation = True

        return input

    def is_valid_input_string(self, input_string: str) -> bool:
        conditions = [input_string[0] == "{", "115,!" in input_string]
        return all(conditions)

    def build_cmd_str(self, address: str, command: str, args: list = None):
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
            logger.warning(f"{command} not a valid command, must be one of {valid_commands}")

        if args:
            args = ','.join(map(str, args))
        else:
            args = ''
        return f"{address} {cmd},{args},!\n"

    def start_config_observer(self):
        self.config_observer = Observer()
        event_handler = config_handler(self.read_config_json)
        self.config_observer.schedule(event_handler, path=self.config_dir, recursive=True)
        self.config_observer.start()

    def parse_config(self, config_dict):
        args_queue = []
        for channel_id in config_dict.keys():
            try:
                output = self.Outputs.get_by_id(channel_id)
                current_control = output.control_mode
            except KeyError:
                print(f"{channel_id} not an available channel, must be one of {self.Outputs._dict.keys()}")
            else:
                match config_dict[channel_id]:
                    case {"mode": "MANUAL", "value": value}:
                        if isinstance(value, int):
                            new_control = manual_control(output.channel, value)
                            if not new_control == current_control:
                                args_queue.append([output.address, new_control.get_args()])
                                output.control_mode = new_control
                        else:
                            print(f"In {channel_id}, value must be of type(int)")

                    case {"mode": "TIMER", "value": value, "time_on": time_on, "time_off": time_off}:
                        if all([isinstance(time_on, int), isinstance(time_off, int), isinstance(value, int)]):
                            time_on = time_on * self.samples_per_second
                            time_off = time_off * self.samples_per_second
                            new_control = timer_control(output.channel, time_on, time_off, value)
                            if not new_control == current_control:
                                args_queue.append([output.address, new_control.get_args()])
                                output.control_mode = new_control
                        else:
                            print(f"In {channel_id}, value/time_on/time_off must be of type(int)")

                    case {"mode": "PID", "variable": variable, "setpoint": setpoint}:
                        if isinstance(setpoint, (float, int)):
                            try:
                                input = self.Inputs.get_by_id(variable)
                            except KeyError:
                                print(f"In {channel_id} variable={variable} must be one of {self.Inputs._dict.keys()}")
                            else:
                                new_control = pid_control(output.channel, input.channel, setpoint)
                                if not new_control == current_control:
                                    if output.address == input.address:
                                        args_queue.append([output.address, new_control.get_args()])
                                        output.control_mode = new_control
                                    else:
                                        print(f"In {channel_id}, control variable must be on same board as output channel")
                        else:
                            print(f"In {channel_id}, setpoint must be of type([float,int])")

                    case {"mode": "ONOFF", "variable": variable, "lower_bound": lower_bound, "upper_bound": upper_bound, "value": value}:
                        if all([isinstance(lower_bound, (float, int)), isinstance(upper_bound, (float, int)), isinstance(value, int)]):
                            try:
                                input = self.Inputs.get_by_id(variable)
                            except KeyError:
                                print(f"In {channel_id} variable={variable} must be one of {self.Inputs._dict.keys()}")
                            else:
                                new_control = onoff_control(output.channel, input.channel, lower_bound, upper_bound, value)
                                if not new_control == current_control:
                                    if output.address == input.address:
                                        args_queue.append([output.address, new_control.get_args()])
                                        output.control_mode = new_control
                                    else:
                                        print(f"In {channel_id}, control variable must be on same board as output channel")
                        else:
                            print(f"In {channel_id}, lower_bound/upper_bound must be of type([float,int])")

                    case {"mode": mode}:
                        print(f"{mode} mode must be on of {valid_control_modes}")

                    case _:
                        print(f"{config_dict[channel_id]}")

        return args_queue

    def reconnect(self):

        time.sleep(1)
        self.serial_port.close()
        self.serial_port = None
        time.sleep(2)
        logger.warning("Serial Port Dissconected")
        logger.warning('Opening connection')
        while self.serial_port is None:
            time.sleep(5)

            try:
                available_ports = get_available_serial_ports()
                print(available_ports)
                self.open_connection(self.port_name)
            except SerialException:
                logger.warning(f"Unsuccessfull: {get_available_serial_ports()}")

        self.serial_port.flushInput()
        self.serial_port.flushOutput()
        logger.warning("Device Reconneted")
        time.sleep(0.5)


class manual_control(NamedTuple):
    control_type = valid_control_modes["MANUAL"]
    channel: int
    value: int

    def get_args(self):
        return [self.control_type, *self]


class timer_control(NamedTuple):
    control_type = valid_control_modes["TIMER"]
    channel: int
    time_on: int
    time_off: int
    value: int

    def get_args(self):
        return [self.control_type, *self]


class pid_control(NamedTuple):
    control_type = valid_control_modes["PID"]
    channel: int
    variable: int
    setpoint: float

    def get_args(self):
        return [self.control_type, *self]


class onoff_control(NamedTuple):
    control_type = valid_control_modes["ONOFF"]
    channel: int
    variable: int
    lower_bound: float
    upper_bound: float
    value: int

    def get_args(self):
        return [self.control_type, *self]


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


def validate_control_mode(mode: str) -> bool:
    if mode not in valid_control_modes:
        raise ValueError(f"control_mode must be one of {valid_control_modes}")
