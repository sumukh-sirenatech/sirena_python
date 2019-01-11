"""
Contains class for LUCI Communication,Robot Communication over LUCI
"""

# Authors: Sumuk,Aadrash

from __future__ import division
import socket
import time
import signal
from threading import Thread, Lock, Condition
import sys
from struct import *
import array
import numpy as np
try:
    import cv2
except ImportError:
    cv2 = None
import time
BUFFER_SIZE = 1024


class LuciCommunication:
    """
    LUCI is a protocol over TCP used for controlling device.
    """
    def __init__(self, ip, port):
        """
        Parameters
        ----------
        ip : string
            IP address of the device.
        port: int
            Port number of socket.
        """
        self.ip = ip
        self.port = port
        self.thread = 0
        self.luci_dict = {}
        self.luci_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def __enter__(self):
        """
        Establishes Luci Commincation with device.
        Creates thread for receiving LUCI Packet.
        Sends Register Command on LUCI.
        Returns
        -------
        Object
        """
        self.thread = 1
        try:
            self.luci_sock.bind(('', 0))
            self.luci_sock.connect((self.ip, self.port))
            print("Device connected")
            self.sendLUCI_data([], 3)
            print("Luci connection Successfull")
            threadoutput = Thread(target=self.Luci_recv)
            threadoutput.daemon = True
            threadoutput.start()
            return self
        except:
            print("here")
            self.thread = 0
            return 0

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Send Deregister Command over LUCI.
        Disconnects the TCP socket.
        """
        self.thread = 0
        try:
            print("here1")
            time.sleep(1)
            self.sendLUCI_data([], 4)
            print("Luci connection Disconnected")
            self.luci_sock.close()
        except:
            pass

    def sendLUCI_data(self, data, boxid):
        """
        It Prepare the message in ByteArray Format.
        Send the Packet with LUCI Header to Device.
        See LUCI Document for more information.
        Parameters
        ----------
        data : list
            list is of number from 0-255 or characters.
        boxid : int
            This reffers to Command in LUCI Communication.
            Should be at in range 0-65536.
        """
        luciarr = bytearray([0, 0, 2, 0, 0, 0, 0, 0])
        luciarr[3] = boxid & 0xff
        luciarr[4] = boxid >> 8 & 0xff
        datalen = len(data)
        datalenL = datalen & 0xff
        datalenH = datalen >> 8 & 0xff
        datalenarr = bytearray([datalenL, datalenH])
        buf = bytearray()
        buf += luciarr
        buf += datalenarr
        buf += bytearray(data)
        packbuf = pack('B' * len(buf), *tuple(buf))
        if(self.luci_sock):
            self.luci_sock.sendall(packbuf)

    def paser_luci(self, data_list):
        """
        Extracts data from LUCI packet
        Parameters
        ----------
        data_list: list
            list contain tcp packet converted from bytearray
        Returns
        -------
        boxid : int
            LUCI Command ID
        data : list
            List containg the data from LUCI Packet
        data_list : list
            TCP is stream so input can contain more than one packet.
            In order to process next Packet Extra bytes are returned.
        """
        if(len(data_list) < 0):
            return(0, 0, [])
        else:
            boxid = data_list[4] + data_list[3] * 256
            data_length = data_list[9] + data_list[8] * 256
            data = data_list[10:10 + data_length]
            return(boxid, data, data_list[10 + data_length:])

    def update_dict(self, luci_dict, boxid, data):
        """
        Given the Command Id it updates the LUCI Packet Dictionary.
        Dictionary Can be used by others to get the data.
        Parameters
        ----------
        luci_dict : Dictionary
            Contain the command ids as key and data as value.
        boxid : int
            LUCI Command id which will reffered by everyone.
        data : list
            Data recived from device after removing LUCI Header.
        """
        if boxid in luci_dict:
            luci_dict[boxid] = data
        else:
            luci_dict[boxid] = data

    def Luci_recv(self):
        """
        This Function is will Run in thread to receive data asynchronously.
        Thread is create in entry function of LUCI Communication class.
        """
        data_list = []
        while(self.thread):
            recv_data = self.luci_sock.recv(BUFFER_SIZE)
            if len(recv_data):
                del data_list[:]
                for i in recv_data:
                    data_list.append(unpack("B", i)[0])
                while(len(data_list)):
                    boxid, data, data_list = self.paser_luci(data_list)
                    if len(data):
                        self.update_dict(self.luci_dict, boxid, data)
        self.threadkilled = 0


class RobotCommunication(LuciCommunication):
    """
    This class send peripherals data over LUCI.
    To Control UART,I2C device a separate header is added over LUCI data.

    Refference :
        luci packet structures Document in gdrive.
    """
    def __init__(self, ip, port):
        """
        Parameters
        ----------
        ip : string
            IP address of the device.
        port: int
            Port number of socket.
        """
        LuciCommunication.__init__(self, ip, port)
        self.command = {"uart_write": 0, "uart_read": 1,
                        "i2cwrite": 2, "gpio": 3, "i2cread": 4}
        self.gpio_command = {"direction": 0, "write": 1, "read": 2}
        self.gpio_direction = {"input": 0, "output": 1}
        self.gpio_value = {"low": 0, "high": 1}
        self.baudrate = [2000000, 1000000, 500000,
                         222222, 117647, 100000, 57142, 9615]

    def __enter__(self):
        """
        Calls Luci class entry function to establish luci communication.
        """
        LuciCommunication.__enter__(self)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Call Luci class exit function.
        """
        LuciCommunication.__exit__(self, exc_type, exc_val, exc_tb)

    def sendoverluci(self, mode, packet0=[], packet1=[]):
        """
        This creates the extra header added to luci data beforing sending over
        LUCI
        Parameters
        ----------
        mode : int
            used to identify uart read/write,i2c read/write,gpio control.
        packet0 : list
            Frist packet to be sent over 254 message box.
        packet1 : list
            Second packet to be sent over 254 message box.
        """
        data = []
        data.append(self.command[mode])
        len_packet0 = len(packet0)
        len_packet1 = len(packet1)
        data.append(len_packet0 & 0xff)
        data.append(len_packet0 >> 8 & 0xff)
        data.append(len_packet1 & 0xff)
        data.append(len_packet1 >> 8 & 0xff)
        if(len_packet0):
            data.extend(packet0)
        if(len_packet1):
            data.extend(packet1)
        # print(data)
        self.sendLUCI_data(data, 254)

    def i2cwrite(self, address, i2c_data):
        """
        This takes list of address and data to be sent over 254 message box.
        Parameters
        ----------
        address : list
            list contains the 7bit address of i2c device.
        i2c_data : list
            list of data,data is list so this will list containg list.
        """
        data = []
        no_of_device = len(address)
        if(no_of_device > 1):
            data.append(1)
        else:
            data.append(0)
        data.append(no_of_device)
        for add in range(no_of_device):
            data.append(address[add])
            data.append(len(i2c_data[add]))
            data.extend(i2c_data[add])
        print(data)
        self.sendoverluci("i2cwrite", data)

    def i2cread(self, address, i2c_register, read_len):
        """
        This function is used to read i2c device connected to device
        Parameters
        ----------
        address : list
            list of 7bit address of device
        i2c_register : list
            data to be sent to read from device to set the pointer in device
        read_len : list
            length of data to be read for each device.
        """
        data = []
        no_of_device = len(address)
        if(no_of_device > 1):
            data.append(1)
        else:
            data.append(0)
        data.append(no_of_device)
        for add in range(no_of_device):
            data.append(address[add])
            data.append(read_len[add])
            data.append(len(i2c_register[add]))
            data.extend(i2c_register[add])
        print(data)
        self.sendoverluci("i2cread", data)

    def gpio_handler(self, mode, gpio, value):
        """
        This Function is used to control gpio on board.
        Parameters
        ----------
        mode : int
            This set the operation on the gpio like direction control or
            read/write.
        gpio : int
            gpio number from 0-4.
        value : int
            value is 0-1. class has several dict for this.
        """
        data = []
        no_of_gpio = len(gpio)
        data.append(self.gpio_command[mode])
        data.append(no_of_gpio)
        for add in range(no_of_gpio):
            data.append(gpio[add])
            data.append(value[add])
        print(data)
        self.sendoverluci("gpio", data)

    def uart_write(self, baudrate, uart_data):
        """
        Used to send UART Protocol Data.
        Parameters
        ----------
        baudrate : int
            index value of the baudrate list in class
        uart_data : list
            list contaning uart data.
        """
        data = []
        data.append(0)
        data.append(self.baudrate.index(baudrate))
        data.extend(uart_data)
        # print(data)
        self.sendoverluci("uart_write", data)

    def uart_read(self, baudrate, comm_index, read_byte, uart_data):
        """
        Used to send UART Protocol Data to read from device.
        Parameters
        ----------
        baudrate : int
            index value of the baudrate list in class
        comm_index : int
            specify motor protocol version 1 or 2
        read_byte : int
            number of bytes to read from motors
        uart_data : list
            list contaning uart data.
        """
        data = []
        data.append(0)
        data.append(self.baudrate.index(baudrate))
        no_of_device = len(read_byte)
        data.append(no_of_device)
        data.append(comm_index)
        for add in range(no_of_device):
            data.append(read_byte[add])
            data.append(len(uart_data[add]))
            data.extend(uart_data[add])
        self.sendoverluci("uart_read", data)


class motor:
    """
    Controls the i2c based motor driver on
    """
    def __init__(self, option):
        """
        Parameters
        ----------
        option : int
            take value 1-4 corresponding to i2c motor drive.
        """
        self.address = 0x60
        self.i2c_address = {1: 0x60, 2: 0x64, 3: 0x62, 4: 0x68}
        self.direction = {"stop": 0x00, "forward": 0x01, "backward": 0x02}
        self.address = self.i2c_address[option]

    def motor_packet(self, speed, direction):
        """
        Control the motor.
        Parameters
        ----------
        speed : int
            value in range 0-100.
        direction : string
            allowed values are stop,forward,backward.
        """
        motor_vel = int(6 + (speed / 100.0) * 57)
        return [(self.direction[direction] & 0x03) | ((motor_vel & 0x3f) << 2)]


class dynamixel:
    """
    Prepare dynamixel packet to be sent over luci to control motor.
    """
    def __init__(self):
        self.AX12 = 0
        self.AX18 = 1
        self.MX28 = 2
        self.MX64 = 3
        self.MX106 = 4
        self.XL320 = 5
        self.BAUDRATES = [2000000, 1000000, 500000,
                          222222, 117647, 100000, 57142, 9615]
        self.dynamixel_data = []

    def getLHbytes(self, num):
        """
        Gives lower and higher byte of 16bit number.
        Parameters
        ----------
        num : int
            16bit number
        Return
        ------
        numL : int
            8bit value of lower byte
        numH : int
            8bit Value of higher byte
        """
        numL = num & 0xff
        numH = (num >> 8) & 0xff
        return numL, numH

    def LUCI_WriteUARTPacket(self, baudRate, uartPacket):
        """
        Makes packet to be sent over luci.
        Parameters
        ----------
        baudRate : index value
            index value of baudrate present in list BAUDRATES
        uartPacket : array
            array of data to be sent to dynamixel motor
        Returns
        ------
        LWUP : array
            array used to sent over luci
        """
        baudRateIndex = min(range(len(self.BAUDRATES)),
                            key=lambda i: abs(self.BAUDRATES[i] - baudRate))
        # Internal Mode = 0, BaudRate Index
        LWUP = array.array('B', [0, baudRateIndex]).tostring()
        LWUP += uartPacket
        return LWUP

    def LUCI_createPacket(self, mode, packet0, packet1):
        """
        Prepare the LUCI data header with packet0 and packet1.
        Parameters
        ----------
        mode : int
            value for uart write/read
            see documentation of luci packet structure
        packet0 : array
            array of bytes
        packet1 : array
            array of bytes
        Returns
        -------
        LuciPacket : array
            array of byte with header
        """
        L0L, L0H = self.getLHbytes(len(packet0))
        L1L, L1H = self.getLHbytes(len(packet1))
        LuciLength = len(packet0) + len(packet1) + 5
        LuciLengthL, LuciLengthH = self.getLHbytes(LuciLength)
        LuciPacket = array.array('B', [mode, L0L, L0H, L1L, L1H]).tostring()
        LuciPacket += packet0
        if(len(packet1) > 0):
            LuciPacket += packet1
        return LuciPacket

    def SyncWriteAxMxPacket(self, lenOfData, startAddress, ids, data):
        """
        Creates sync write packet of dynamixel.
        Parameters
        ----------
        lenOfData : int
            number of bytes for each motor
        startAddress : int
            register address of dynamixel motor
        ids : list
            list od motor ids
        data : list
            data to be sent for each motor
        Returns
        -------
        syncPacket : list
            list with sync packet header added
        """
        packetLength = (lenOfData + 1) * (len(ids)) + 4
        syncPacket = [255, 255, 254, packetLength,
                      131, startAddress, lenOfData]
        syncPacket.extend(data)
        crc = 255 - ((sum(syncPacket) - 255 - 255) & 0xff)
        syncPacket.extend([crc])
        return syncPacket

    def writeDegreesSyncAxMx(self, ids, motorTypes, degrees, rpm, baudRate):
        """
        Prepares Sync write packet for dynamixel motor
        Parameters
        ----------
        ids : list
            list of motor ids
        motorTypes : list
            list of motor type
        degrees : list
            list of goal position for each motor in degrees
        rpm : list
            list of goal rpm for each motor
        baudrate : int
            index value for baudrate
        Returns
        -------
        syncPacket : list
            list in 8bit format for sync write command
        """
        data = []
        for i, mid in enumerate(ids):

            if(motorTypes[i] == self.AX12 or motorTypes[i] == self.AX18):
                goalPos = int(round((degrees[i] / 300.0) * 1023.0))
                goalVel = int(round((rpm[i] / 114.0) * 1023.0))
                goalPosL, goalPosH = self.getLHbytes(goalPos)
                goalVelL, goalVelH = self.getLHbytes(goalVel)
                data.extend([mid, 0X80, 0X80, goalPosL,
                             goalPosH, goalVelL, goalVelH])
            elif(motorTypes[i] == self.MX28 or
                 motorTypes[i] == self.MX64 or
                 motorTypes[i] == self.MX106):
                goalPos = int(round((degrees[i] / 360.0) * 4095.0))
                goalVel = int(round((rpm[i] / 117.0) * 1023.0))
                goalPosL, goalPosH = self.getLHbytes(goalPos)
                goalVelL, goalVelH = self.getLHbytes(goalVel)
                data.extend([mid, goalPosL, goalPosH, goalVelL, goalVelH])
            else:
                return

        syncPacket = self.SyncWriteAxMxPacket(6, 28, ids, data)
        return syncPacket

    def setTorqueEnableSyncAxMx(self, ids, states, baudRate):
        """
        Set the torque enable register in motor
        Parameters
        ----------
        ids : list
            list of motor ids
        states : list
            list of state value for each motor 0-1
        baudrate : int
            index value for baudrate
        """
        data = []
        for i, mid in enumerate(ids):
            data.extend([mid, states[i]])
        syncPacket = self.SyncWriteAxMxPacket(1, 25, ids, data)
        return syncPacket

    def update_crc(self, packet):
        """
        Dynamixel protocol version 2 has different crc calaculation.
        crc_table is used to calculate the crc
        For more information refer dynamixel website
        Parameters
        ----------
        packet : list
            list of 8bit value
        Returns
        crc_accum : 16bit int
            accumulated crc for packet
        """
        crc_accum = 0
        crc_table = [0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014,
                     0x8011, 0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D,
                     0x8027, 0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078,
                     0x807D, 0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A,
                     0x804B, 0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC,
                     0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5,
                     0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0,
                     0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
                     0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087,
                     0x0082, 0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D,
                     0x8197, 0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB,
                     0x01AE, 0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA,
                     0x81FB, 0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC,
                     0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145,
                     0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173,
                     0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
                     0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137,
                     0x0132, 0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E,
                     0x0104, 0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318,
                     0x831D, 0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A,
                     0x832B, 0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F,
                     0x036A, 0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356,
                     0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0,
                     0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
                     0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7,
                     0x03E2, 0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD,
                     0x83B7, 0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B,
                     0x038E, 0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A,
                     0x829B, 0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC,
                     0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6,
                     0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0,
                     0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
                     0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257,
                     0x0252, 0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E,
                     0x0264, 0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B,
                     0x023E, 0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219,
                     0x0208, 0x820D, 0x8207, 0x0202

                     ]

        for j, byte in enumerate(packet):
            i = ((crc_accum >> 8) ^ byte) & 0xff
            crc_accum = (crc_accum << 8) ^ crc_table[i]

        return crc_accum

    def SyncWriteXlPacket(self, lenOfData, startAddress, ids, data):
        """
        Creates sync write packet of dynamixel.
        Parameters
        ----------
        lenOfData : int
            number of bytes for each motor
        startAddress : int
            register address of dynamixel motor
        ids : list
            list od motor ids
        data : list
            data to be sent for each motor
        Returns
        -------
        syncPacket : list
            list with sync packet header added
        """
        packetLength = (lenOfData + 1) * len(ids) + 7
        packetLengthL, packetLengthH = self.getLHbytes(packetLength)
        startAddressL, startAddressH = self.getLHbytes(startAddress)
        lenOfDataL, lenOfDataH = self.getLHbytes(lenOfData)
        syncPacket = [255, 255, 253, 0, 254, packetLengthL, packetLengthH,
                      131, startAddressL, startAddressH, lenOfDataL,
                      lenOfDataH]
        syncPacket.extend(data)
        crc = self.update_crc(syncPacket)
        crcL, crcH = self.getLHbytes(crc)
        syncPacket.extend([crcL, crcH])
        return syncPacket

    def writeDegreesSyncXl(self, ids, motorTypes, degrees, rpm, baudRate):
        """
        Prepares Sync write packet for dynamixel motor
        Parameters
        ----------
        ids : list
            list of motor ids
        motorTypes : list
            list of motor type
        degrees : list
            list of goal position for each motor in degrees
        rpm : list
            list of goal rpm for each motor
        baudrate : int
            index value for baudrate
        Returns
        -------
        syncPacket : list
            list in 8bit format for sync write command
        """
        data = []
        for i, mid in enumerate(ids):
            if(motorTypes[i] == self.XL320):
                goalPos = int(round((degrees[i] / 300.0) * 1023.0))
                goalVel = int(round((rpm[i] / 114.0) * 1023.0))
                goalPosL, goalPosH = self.getLHbytes(goalPos)
                goalVelL, goalVelH = self.getLHbytes(goalVel)
                data.extend([mid, goalPosL, goalPosH, goalVelL, goalVelH])
            else:
                return

        syncPacket = self.SyncWriteXlPacket(4, 30, ids, data)
        return syncPacket

    def setPIDSyncXl(self, ids, pid, baudRate):
        """
        Set the pid values in motor
        Parameters
        ----------
        ids : list
            list of motor ids
        pid : list
            list of pid value for each motor which is itself list of pid values
        baudrate : int
            index value for baudrate
        """
        data = []
        for i, mid in enumerate(ids):
            data.extend([mid, pid[2], pid[1], pid[0]])
        syncPacket = self.SyncWriteXlPacket(3, 27, ids, data)
        return syncPacket


class snake_led:
    """
    Control the color of leds on snake which is a i2c device.
    """
    def __init__(self):
        """
        Set the address of i2c led driver.
        """
        self.address = 0x61

    def set_pwm(self, data, color):
        """
        Led Driver has 4 state for led.
        On,Off,PWM1 & PWM2.
        Parameters
        ----------
        data : list
            data in list to be sent to led driver.
        color : int
            8bit for each color.
        """
        if color == 0:
            return 1
        elif color == 255:
            return 0
        else:
            if data[2] == 0:
                data[2] = color
                return 2
            else:
                if data[2] == color:
                    return 2
                else:
                    if data[4] == 0:
                        data[4] = color
                        return 3
                    else:
                        return 3

    def make_color(self, red, blue, green):
        """
        Takes the RGB values of led and makes packet to be sent over luci.
        Parameters
        ----------
        red : int
            8bit value for red color
        blue : int
            8bit value for blue color
        green : int
            8bit value for green color
        """
        data = [0x11, 0, 0, 0, 0, 0, 0]
        rstate = self.set_pwm(data, red)
        gstate = self.set_pwm(data, green)
        bstate = self.set_pwm(data, blue)
        print(bstate)
        print(rstate)
        print(gstate)
        data[2] = 255 - data[2]
        data[4] = 255 - data[4]
        data[5] = 64 + bstate * 16 + rstate * 4 + gstate
        data[6] = 64 + bstate * 16 + rstate * 4 + gstate
        return data


class nino_led:
    """
    Led driver on Nino is i2c led drive this class enable to
    control color over luci.
    """
    def __init__(self):
        """
        Led Driver 7bit address
        """
        self.address = 0x60

    def set_pwm(self, data, color):
        """
        Led Driver has 4 state for led.
        On,Off,PWM1 & PWM2.
        Parameters
        ----------
        data : list
            data in list to be sent to led driver.
        color : int
            8bit for each color.
        """
        if color == 0:
            return 1
        elif color == 255:
            return 0
        else:
            if data[2] == 0:
                data[2] = color
                return 2
            else:
                if data[2] == color:
                    return 2
                else:
                    if data[4] == 0:
                        data[4] = color
                        return 3
                    else:
                        return 3

    def make_color(self, red, blue, green):
        """
        Takes the RGB values of led and makes packet to be sent over luci.
        Parameters
        ----------
        red : int
            8bit value for red color
        blue : int
            8bit value for blue color
        green : int
            8bit value for green color
        """
        data = [0x12, 0, 0, 0, 0, 0, 0, 0]
        rstate = self.set_pwm(data, red)
        gstate = self.set_pwm(data, green)
        bstate = self.set_pwm(data, blue)
        print(bstate)
        print(rstate)
        print(gstate)
        data[2] = 255 - data[2]
        data[4] = 255 - data[4]
        data[5] = bstate * 64 + rstate * 16 + gstate * 4 + bstate
        data[6] = gstate * 64 + bstate * 16 + rstate * 4 + gstate
        data[7] = rstate * 64 + gstate * 16 + bstate * 4 + rstate
        return data


class nino_nled:
    """
    Contol the nino eye color over LUCI.
    """
    def __init__(self):
        """
        Nino eye is i2c based driver.Setting the i2c 7bit address.
        """
        self.address = 0x3e

    def set_reg(self, reg, data):
        """
        Makes list with data and register address.
        Parameters
        ----------
        register : int
            i2c register which is reffered.
        data : list
            data to sent to i2c register.
        """
        data1 = []
        data1.append(reg)
        data1.extend(data)
        return data1

    def led_init(self):
        """
        Led driver needs to be intilized with certain value done in
        this function.
        """
        list_data = []
        # disable input buffer
        list_data.append(self.set_reg(0x00, [0xff, 0xff]))
        list_data.append(self.set_reg(0x06, [0x00, 0x00]))  # disable pulldown
        list_data.append(self.set_reg(0x08, [0x00, 0x00]))  # disable pullup
        list_data.append(self.set_reg(0x0a, [0xff, 0xff]))  # enable open drain
        list_data.append(self.set_reg(0x0e, [0x00, 0x00]))  # enable output
        # write high on output
        list_data.append(self.set_reg(0x10, [0xff, 0xff]))
        list_data.append(self.set_reg(0x1e, [0x40, 0xc8]))  # clock config
        list_data.append(self.set_reg(0x20, [0xff, 0xff]))  # set led driver
        # write low on output
        list_data.append(self.set_reg(0x10, [0x00, 0x00]))
        return list_data

    def set_color(self, red, green, blue):
        """
        Change to color according to rgb value.
        Parameters
        ----------
        red : int
            8bit value for red color
        green : int
            8bit value for green color
        blue : int
            8bit value for blue color
        """
        list_data = []
        list_data.append(self.set_reg(0x2a, [red]))
        list_data.append(self.set_reg(0x2d, [green]))
        list_data.append(self.set_reg(0x30, [blue]))

        list_data.append(self.set_reg(0x36, [red]))
        list_data.append(self.set_reg(0x3b, [green]))
        list_data.append(self.set_reg(0x40, [blue]))

        list_data.append(self.set_reg(0x4a, [red]))
        list_data.append(self.set_reg(0x4d, [green]))
        list_data.append(self.set_reg(0x50, [blue]))

        list_data.append(self.set_reg(0x56, [red]))
        list_data.append(self.set_reg(0x5b, [green]))
        list_data.append(self.set_reg(0x60, [blue]))
        return list_data

    def gen_add(self, list_data):
        """
        Generates list of same address with length of list_data.
        Parameter
        ---------
        list_data : list
            list containg list of register values to be set.
        Return
        ------
        list of address and length equal to len of list_data.
        """
        return [self.address] * len(list_data)
