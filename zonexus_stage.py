# -*- coding: utf-8 -*-

import serial
import time


class Zonexus_Continuous_Rotation_Stage_Control():
    def __init__(self, zncom='COM3'):

        print('Connecting to Zonexus stage')
        self.ser = serial.Serial(zncom,
                                 baudrate=115200,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 bytesize=serial.EIGHTBITS)

        if self.ser.is_open:
            self.ser.close()

        self.sendCommand('X1Y13,45') # Set encoder type
        self.sendCommand('X1Y6,1') # Set negative counting (important!)
        self.sendCommand('X1Y3,-65536') # Set stage lower limit (= 2 full rotations of stage)
        self.sendCommand('X1Y4,65536') # Set stage upper limit (= 2 full rotations of stage)

    def sendCommand(self, command):
        """
        Send command to Zonexus stage

        Parameters
        ----------
        command : str
            Command to be sent to the Zonexus stage

        Returns
        -------
        datastr: str
            Response from the Zonexus stage
        """
        self.ser.open()

        command = command + '\r'
        data_to_send = bytes([ord(ii) for ii in command])

        data = -1
        if not self.ser.is_open:
            print('Serial port is not open')
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        try:
            self.ser.write(data_to_send)
        except:
            print('Send Error')

        data = b''
        try:
            while(data[-1:]!=b'\r'):
                data = data + self.ser.read(1)
            data = data[:-1]
        except:
            print('Read Error')

        self.ser.close()
        datastr = data.decode()

        return datastr

    def readEncoder(self):
        """
        Read the Zonexus stage encoder

        Returns
        -------
        deg : float
            encoder alpha in degrees
        """
        reply = self.sendCommand('X1E') # command to read encoder
        enc = int(reply[4:])
        return enc

    def getAlpha(self):
        """
        Find alpha of the Zonexus stage

        Returns
        -------
        deg : float
            alpha in degrees
        """
        enc = self.readEncoder()
        deg = enc*360/(2**15) # encoder reads 2^15 bits for one revolution
        print('alpha =', deg)
        return deg

    def setAlpha(self, deg, WAIT=True, dencmax=5):
        """
        Set the Zonexus stage alpha

        Parameters
        ----------
        deg : float
            target alpha in degrees
        """
        self.sendCommand('X1M2') # Unpark stage using delta waveform
        target_enc = int(deg*(2**15)/360) # Calculate target encoder position
        self.sendCommand('X1T{},300'.format(target_enc))
        ''' WAIT FOR STAGE TO FINISH MOVING '''
        if WAIT:
            MOVING = True
            while MOVING:
                time.sleep(1)
                current_enc = self.readEncoder()
                if abs(target_enc - current_enc) < dencmax:
                    MOVING = False
                    print('Stage at target position')

    def stopMotor(self, event):
        self.sendCommand('X1S')

    def parkMotor(self, event):
        self.sendCommand('X1M4')

    def unparkMotor(self, event):
        self.sendCommand('X1M2')
