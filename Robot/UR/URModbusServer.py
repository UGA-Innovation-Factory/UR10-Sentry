from Communication.ModbusTCP import ModbusTCP

import time, math, struct

# The robot controller acts as a Modbus TCP server (port 502),
# clients can establish connections to it and send standard MODBUS requests to it.

# - Note that some Modbus device manufacturers use the terms Master (client) and Slave (server).
# Typically the external IO device is going to be a server and the robot is going to behave as the client
# (requesting and consuming messages from the server). Master=client and Slave=server.
# However, note that the UR controller can be both a server and a client

# - Note that all values are unsigned, if you want to convert to signed integers,
# program "if (val > 32768): val = val - 65535".
#
# - The MODBUS Server has 0-based addressing.
# Be aware that some other devices are 1-based (e.g. Anybus X-gateways), then just add one to the address
# on that device. (e.g. address 3 on the robot will be address 4 on the Anybus X-gateway)


class URModbusServer:
    """Give read and write access to data in the robot controller for other devices

    An interface for communicating with the modbus TCP server (port 502) on the UR.
    Defines functions for retrieving information from the controller.
    Information will be re-requested if an error occurs.
    All information will be formatted to human readable information.
    """

    def __init__(self, host):
        """
        :param host: IP address to connect with
        """
        self.modbusTCP = ModbusTCP(host, 502)

    def get_tcp_position(self):
        """
        Connects with the Modbus server to requests Cartesian data of the TCP
        :return: Readable cartesian data of TCP, vector in mm, axis in radials
        """
        packet = self.modbusTCP.read_holding_registers(400, quantity=6)

        if packet is None:
            time.sleep(0.5)
            print("[TCP] Modbus Error: retrying")
            return self.get_tcp_position()
        else:
            x = self._format(packet[9:11]) / 10
            y = self._format(packet[11:13]) / 10
            z = self._format(packet[13:15]) / 10
            rx = self._format(packet[15:17]) / 1000
            ry = self._format(packet[17:19]) / 1000
            rz = self._format(packet[19:21]) / 1000
            return x, y, z, rx, ry, rz
        
    def get_joint_angles(self, tries=0) -> tuple:
        """
        Connects with the Modbus server to requests the angles of each joint, in radians
        :return: Readable angle values of each joint in radials
        """
        if tries>10:
            print("[Angles] Modbus Error: Failed")
            return 0, 0, 0, 0, 0, 0

        packet = self.modbusTCP.read_holding_registers(270, quantity=6)
        packet_signs = self.modbusTCP.read_holding_registers(320, quantity=6)

        if (packet is None) or (packet_signs is None):
            time.sleep(0.01)
            print(f"[Angles] Modbus Error #{tries}: retrying")
            return self.get_joint_angles(tries+1)
        else:
            base = self._format_sign(packet[9:11], packet_signs[9:11])
            shoulder = self._format_sign(packet[11:13], packet_signs[11:13])
            elbow = self._format_sign(packet[13:15], packet_signs[13:15])
            wrist_1 = self._format_sign(packet[15:17], packet_signs[15:17])
            wrist_2 = self._format_sign(packet[17:19], packet_signs[17:19])
            wrist_3 = self._format_sign(packet[19:21], packet_signs[19:21])
            
            return base, shoulder, elbow, wrist_1, wrist_2, wrist_3

    def get_joint_angles_degrees(self, tries=0):
        angles = self.get_joint_angles(tries)
        return tuple([round(math.degrees(angle), 3) for angle in angles])

    def get_joint_speeds(self, tries=0):
        """
        Connects with the Modbus server to requests the speed of each joint, in radians per second
        :return: Readable angle speeds of each joint in radians per second
        """
        if tries>10:
            print("[Speeds] Modbus Error: Failed")
            return 0, 0, 0, 0, 0, 0

        packet = self.modbusTCP.read_holding_registers(280, quantity=6)

        if (packet is None):
            time.sleep(0.01)
            print(f"[Speeds] Modbus Error#{tries}: retrying")
            return self.get_joint_speeds(tries+1)
        else:
            base = self._format(packet[9:11]) / 1000
            shoulder = self._format(packet[11:13]) / 1000
            elbow = self._format(packet[13:15]) / 1000
            wrist_1 = self._format(packet[15:17]) / 1000
            wrist_2 = self._format(packet[17:19]) / 1000
            wrist_3 = self._format(packet[19:21]) / 1000
            
            return base, shoulder, elbow, wrist_1, wrist_2, wrist_3

    @staticmethod
    def _format(d):
        """Formats signed integers to unsigned float

        :param d: signed integer to format
        :return: unsigned float
        """
        d = d.hex()
        d_i = int(d, 16)
        d_f = 0

        if d_i < 32768:
            d_f = float(d_i)
        if d_i > 32767:
            d_i = 65535 - d_i
            d_f = float(d_i) * -1

        return d_f
    
    @staticmethod
    def _format_2(d):
        """Formats signed integers to signed float

        :param d: signed integer to format
        :return: signed float
        """
        d = d.hex()
        d_i = int(d, 16)
        d_f = float(d_i)
        return d_f
    
    @staticmethod
    def _format_sign(d, sign):
        """

        :param d:
        :param sign: 
        :return: 
        """
        sign = sign.hex()
        sign = int(sign, 16)

        d = d.hex()
        d_i = int(d, 16)
        d_f = float(d_i) / 1000

        if sign == 0:
            return d_f
        else:
            d_f = round(d_f - math.radians(360), 3)
            return d_f
