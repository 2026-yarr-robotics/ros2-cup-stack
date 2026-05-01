#!/usr/bin/env python3
"""OnRobot RG2/RG6 Modbus TCP wrapper."""

from pymodbus.client.sync import ModbusTcpClient as ModbusClient


class RG:
    """Minimal RG2/RG6 gripper interface used by cup stacking tasks."""

    def __init__(self, gripper, ip, port):
        self.client = ModbusClient(
            ip,
            port=port,
            stopbits=1,
            bytesize=8,
            parity="E",
            baudrate=115200,
            timeout=1,
        )
        if gripper not in ["rg2", "rg6"]:
            print("Please specify either rg2 or rg6.")
            return
        self.gripper = gripper
        if self.gripper == "rg2":
            self.max_width = 1100
            self.max_force = 400
        elif self.gripper == "rg6":
            self.max_width = 1600
            self.max_force = 1200
        self.open_connection()

    def open_connection(self):
        """Open the connection with a gripper."""

        self.client.connect()

    def close_connection(self):
        """Close the connection with the gripper."""

        self.client.close()

    def get_width(self):
        """Read current width between gripper fingers in 1/10 millimeters."""

        result = self.client.read_holding_registers(
            address=267,
            count=1,
            unit=65,
        )
        return result.registers[0] / 10.0

    def get_status(self):
        """Read current device status as a list of status flags."""

        result = self.client.read_holding_registers(
            address=268,
            count=1,
            unit=65,
        )
        status = format(result.registers[0], "016b")
        status_list = [0] * 7
        if int(status[-1]):
            print("A motion is ongoing so new commands are not accepted.")
            status_list[0] = 1
        if int(status[-2]):
            print("An internal- or external grip is detected.")
            status_list[1] = 1
        if int(status[-3]):
            print("Safety switch 1 is pushed.")
            status_list[2] = 1
        if int(status[-4]):
            print("Safety circuit 1 is activated so it will not move.")
            status_list[3] = 1
        if int(status[-5]):
            print("Safety switch 2 is pushed.")
            status_list[4] = 1
        if int(status[-6]):
            print("Safety circuit 2 is activated so it will not move.")
            status_list[5] = 1
        if int(status[-7]):
            print("Any of the safety switch is pushed.")
            status_list[6] = 1

        return status_list

    def move_gripper(self, width_val, force_val=400):
        """Move gripper to the specified width."""

        params = [force_val, width_val, 16]
        print("Start moving gripper.")
        self.client.write_registers(address=0, values=params, unit=65)
