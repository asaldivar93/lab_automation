#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 2024 Feb 29 12:00

@author: Alexis Saldivar
"""
from typing import Callable

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

# logging.basicConfig(
#     filename="record.log", encoding='utf-8',
#     level=logging.DEBUG, format='%(asctime)s %(levelname)s:%(message)s'
# )

valid_baud_rates = [230400, 115200, 74880, 57600, 38400, 19200, 9600]
valid_commands = {
    "GET_BOARD_INFO": 0, "UPDATE_CONFIGURATION": 1,
}
valid_control_modes = {"MANUAL": 0, "TIMER": 1, "PID": 2, "ONOFF": 3}
valid_input_types = ["adc", "i2c", "spi", "pulses", "flow"]
valid_output_types = {"pwm": range(0, 255), "digital": [0, 1], "stepper": range(0, 1000)}


class Dictlist(list):
    def __init__(self, iterable: list) -> None:
        list.extend(self, iterable)
        self._dict = self._generate_index()

    def _generate_index(self) -> dict:
        return {obj.id: k for k, obj in enumerate(self)}

    def get_by_id(self, id: str):
        return list.__getitem__(self, self._dict[id])


class config_handler(FileSystemEventHandler):
    def __init__(self, fun_callback: Callable):
        self.fun_callback = fun_callback

    def on_modified(self, event):
        self.fun_callback()


class Output():
    def __init__(self, address: str, type: str,
                 channel: float):
        self.address = address
        self.id = "_".join([address, str(channel)])
        self.type = type
        self.channel = channel
        self.bounds = valid_output_types[type]

    def __repr__(self):
        return f"Output(id={self.id}, channel={self.channel}, control_mode={self.control_mode})"


class Input():
    def __init__(self, address: str, type: str, channel: float, variable: str):
        validate_input_type(type)
        self.address = address
        self.id = "_".join([address, str(channel)])
        self.variable = variable
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

        board_info_dict = self.request_board_info()
        self.samples_per_second = board_info_dict["samples_per_second"]

        self.set_outputs(board_info_dict)
        self.set_inputs(board_info_dict)
        print("Connection successfull\n")
        logging.info("Connection successfull\n")

        self.config_dir = "configuration/"
        self.read_config_json()

    def set_serial_port(self, port_name):
        validate_serial_port(port_name)
        self.port_name = port_name

    def set_baud_rate(self, baud_rate):
        validate_baud_rate(baud_rate)
        self.baud_rate = baud_rate

    def open_connection(self):
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
            channels_list = board_info["ins"].keys()
            for type, channel, variable in board_info["inputs"]:
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
                logging.warning(f" JSONDecodeError: {error}")

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
        try:
            return ast.literal_eval(valid_json_string)
        except SyntaxError:
            return {}

    def convert_input_string_to_json(self, input_string: str) -> str:
        return input_string.replace("115,!", "").replace("'", '"').replace(", }", "}").replace("100,!", "")

    def update_configuration(self):
        if self.is_config_updated:
            command = "UPDATE_CONFIGURATION"
            args_queue = self.parse_config(self.config_dict)
            while args_queue:
                address, args = args_queue[0]
                self.write_command(address, command, args)
                args_queue.pop(0)
            self.is_config_updated = False
            print("Board configuration updated")
            logging.info("Board configuration updated")

    def write_command(self, address: str, command: str, args: list = None):
        confirmation = False
        cmd_str = self.build_cmd_str(address, command, args)
        while not confirmation:
            self.serial_port.write(cmd_str.encode())
            self.serial_port.flush()
            # self.serial_port.flushOutput()
            if self.serial_port.inWaiting() > 0:
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
            logging.warning(f"{command} not a valid command, must be one of {valid_commands}")

        if args:
            args = ','.join(map(str, args))
        else:
            args = ''
        return f"{address} {cmd},{args},!\n"

    def start_config_observer(self):
        self.config_observer = Observer()
        event_handler = config_handler(self.read_config_json)
        self.config_observer.schedule(event_handler, path="configuration/", recursive=True)
        self.config_observer.start()

    def parse_config(self, config_dict):
        args_queue = []
        for channel_id in config_dict.keys():
            try:
                output_i = self.Outputs.get_by_id(channel_id)
                out_channel = output_i.channel
                address = output_i.address
                current_control = output_i.control_mode
            except KeyError:
                logging.warning(f"{channel_id} not an available channel, must be one of {self.Outputs._dict.keys()}")
            else:
                match config_dict[channel_id]:
                    case {"mode": "MANUAL", "value": value}:
                        control_mode = valid_control_modes["MANUAL"]
                        if not control_mode == current_control:
                            if isinstance(value, int):
                                args_queue.append([address, [control_mode, out_channel, value]])
                                output_i.control_mode = control_mode
                            else:
                                logging.warning(f"In {channel_id}, value must be of type(int)")

                    case {"mode": "TIMER", "value": value, "time_on": time_on, "time_off": time_off}:
                        control_mode = valid_control_modes["TIMER"]
                        if not control_mode == current_control:
                            if all([isinstance(time_on, int), isinstance(time_off, int), isinstance(value, int)]):
                                time_on = time_on * self.samples_per_second
                                time_off = time_off * self.samples_per_second
                                args_queue.append([address, [control_mode, out_channel, time_on, time_off, value]])
                                output_i.control_mode = control_mode
                            else:
                                logging.warning(f"In {channel_id}, value/time_on/time_off must be of type(int)")

                    case {"mode": "PID", "setpoint": setpoint, "variable": variable}:
                        control_mode = valid_control_modes["PID"]
                        if not control_mode == current_control:
                            try:
                                in_channel = self.Inputs.get_by_id(variable).channel
                            except KeyError:
                                logging.warning(f"In {channel_id} variable={variable} must be one of {self.Inputs._dict.keys()}")
                            else:
                                if isinstance(setpoint, (float, int)):
                                    args_queue.append([address, [control_mode, out_channel, in_channel, setpoint]])
                                    output_i.control_mode = control_mode
                                else:
                                    logging.warning(f"In {channel_id}, setpoint must be of type([float,int])")

                    case {"mode": "ONOFF", "value": value, "variable": variable, "lower_bound": lower_bound, "upper_bound": upper_bound}:
                        control_mode = valid_control_modes["ONOFF"]
                        if not control_mode == current_control:
                            try:
                                in_channel = self.Inputs.get_by_id(variable).channel
                            except KeyError:
                                logging.warning(f"In {channel_id} variable={variable} must be one of {self.Inputs._dict.keys()}")
                            else:
                                if all([isinstance(lower_bound, (float, int)), isinstance(upper_bound, (float, int)), isinstance(value, int)]):
                                    args_queue.append([address, [control_mode, out_channel, in_channel, lower_bound, upper_bound, value]])
                                    output_i.control_mode = control_mode
                                else:
                                    logging.warning(f"In {channel_id}, lower_bound/upper_bound must be of type([float,int])")

                    case {"mode": mode}:
                        logging.warning(f"{mode} mode must be on of {valid_control_modes}")

                    case _:
                        logging.warning(f"{config_dict[channel_id]}")

        return args_queue

    def reconnect(self):

        time.sleep(1)
        self.serial_port.close()
        self.serial_port = None
        time.sleep(2)
        logging.warning("Serial Port Dissconected")
        logging.warning('Opening connection')
        while self.serial_port is None:
            time.sleep(5)

            try:
                self.open_connection()
            except SerialException:
                logging.warning(f"Unsuccessfull: {get_available_serial_ports()}")

        self.serial_port.flushInput()
        self.serial_port.flushOutput()
        logging.warning("Device Reconneted")
        time.sleep(0.5)


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
