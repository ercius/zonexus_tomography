# -*- coding: utf-8 -*-
"""
Created on Fri Sep 15 10:14:26 2023

@author: alexa
"""

import zmq
import numpy as np
import pickle
import argparse
import socket
import json
import pynetstring
import serial
import time

# For connections to FEI TEMScripting and TIA
from comtypes.client import CreateObject
from comtypes.safearray import safearray_as_ndarray
    
def getAlphaDummy():
    return np.random.rand()*360-180

class TIA_control():
    def __init__(self):
        # Connect to the microscope
        self._microscope = CreateObject('TEMScripting.Instrument')
        self.TIA = CreateObject('ESVision.Application')
        self.Acq = self._microscope.Acquisition
        self.Ill = self._microscope.Illumination
        self.Proj = self._microscope.Projection
        self.Stage = self._microscope.Stage
        
        # Connect to STEM
        detector0 = self.Acq.Detectors(0)
        # Add the first detector
        self.Acq.AddAcqDevice(detector0)
        
        self.TIA.ScanningServer().AcquireMode = 1 #0=continuous, 1=single
        self.TIA.ScanningServer().ScanMode = 2 #0=spot, 1=line, 2=frame

    def close_column_valve(self):
        self._microscope.Vacuum.ColumnValvesOpen = False
        print('Column valves closed')

    def create_or_set_display_window(self, sizeX, sizeY):
        self.window_name = 'Zonexus image'
        winlist = self.TIA.DisplayWindowNames()
        found = False
        for ii in range(winlist.count):
            if winlist[ii] == self.window_name:
                found = True
                
        if found:
            self.w2D = self.TIA.FindDisplayWindow(self.window_name)
            self.d1 = self.w2D.FindDisplay('Image 1 Display')
            if self.d1 is not None:
                self.disp = self.d1.Image
            else:
                self.d1 = self.w2D.addDisplay('Image 1 Display', 0,0,3,1)
                self.disp = self.d1.AddImage('Image 1', sizeX, sizeY, self.TIA.Calibration2D(0,0,1,1,0,0))
        else:
            self.w2D = self.TIA.AddDisplayWindow()
            self.w2D.name = self.window_name
            self.d1 = self.w2D.addDisplay('Image 1 Display', 0,0,3,1)
            self.disp = self.d1.AddImage('Image 1', sizeX, sizeY, self.TIA.Calibration2D(0,0,1,1,0,0))

    def set_mag(self, mag):
        self.Ill.StemMagnification = mag
        print('Mag set to {}'.format(self.Ill.StemMagnification))
        
    def get_stage_pos(self):
        ''' Get stage position '''
        stageObj = self.Stage.Position
        print('Stage position = {}'.format(stageObj))
        return stageObj.X, stageObj.Y, stageObj.Z, stageObj.A, stageObj.B

    def move_stage_delta(self, dX=0, dY=0, dZ=0, dA=0):
        ''' Move stage by delta value '''
        n = 15
        print('Moving by {}, {}, {}, {}'.format(dX, dY, dZ, dA))
        stageObj = self.Stage.Position
        stageObj.X += dX
        stageObj.Y += dY
        stageObj.Z += dZ
        stageObj.A += dA
        self.Stage.GoTo(stageObj, n)
        #print('Stage moved to = {}'.format(self.Stage.Position()))

    def blank(self):
        ''' Blanks beam '''
        self.Ill.BeamBlanked = True
        print('Beam blanked')
    
    def unblank(self):
        ''' Unblanks beam '''
        self.Ill.BeamBlanked = False
        print('Beam unblanked')
    
    def change_defocus(self, df):
        '''
        Changes the defocus
        
        Parameters
        ----------
        df : float
            Amount of defocus to change (in metres)
        '''
        print('Changing defocus by {}'.format(df))
        currentDF = self.Proj.Defocus
        self.Proj.Defocus = currentDF + df
        print('Defocus set to {}'.format(self.Proj.Defocus))
        
    def microscope_acquire_image(self, dwell, shape, offset=(0,0)):
        '''
        Acquire image in TIA
        
        Parameters
        ----------
        dwell : float
            Dwell time
        shape : tuple, array
            Image shape
        offset : typle, array
            Offset of image from current center (might be issues if more than one value is non-zero)
        
        Returns
        -------
        image_data : array
            Acquired image
        '''
        
        sizeX = shape[0]
        sizeY = shape[1]
        centerX = offset[0]
        centerY = offset[1]
        
        print('Acquiring image with shape = {}, {}, offset = {}, {}'.format(sizeX, sizeY, centerX, centerY))
        
        if self.TIA.AcquisitionManager().IsAcquiring:
            self.TIA.AcquisitionManager().Stop()
        
        self.create_or_set_display_window(sizeX, sizeY)

        scrange = self.TIA.ScanningServer().GetTotalScanRange

        length = np.maximum(sizeX, sizeY)
        startX = scrange.StartX/length*sizeX
        endX = scrange.EndX/length*sizeX
        startY = scrange.StartY/length*sizeY
        endY = scrange.EndY/length*sizeY
        resolution = (endX-startX)/sizeX
        
        self.TIA.ScanningServer().SetFrameScan(self.TIA.Range2D(startX,startY,endX,endY), resolution) # can resolution be different in x and y?
        self.TIA.ScanningServer().DwellTime = dwell
        
        calX = self.TIA.ScanningServer().ScanResolution
        calY = self.TIA.ScanningServer().ScanResolution
        
        # Needed in case someone runs search between Zonexus searches
        self.TIA.ScanningServer().AcquireMode = 1 #0=continuous, 1=single
        self.TIA.ScanningServer().ScanMode = 2 #0=spot, 1=line, 2=frame
        
        self.TIA.AcquisitionManager().LinkSignal('Analog3', self.d1.Image)
        
        self.unblank()
        self.TIA.AcquisitionManager().Start()
        while self.TIA.AcquisitionManager().IsAcquiring:
            pass
        self.blank()

        data = self.disp.Data
        image_data = np.array(data.Array)
        unit1 = self.d1.SpatialUnit # returns SpatialUnit object
        unitName = unit1.unitstring # returns a string (such as nm)

        return image_data, calX, calY, unitName

    def microscope_acquire_image_old(self, dwell, shape, offset=(0,0)):
        '''
        Acquire image in TIA
        Todo: Remove this once you figure out the TIA window issue
        
        Parameters
        ----------
        dwell : float
            Dwell time
        shape : tuple, array
            Image shape
        offset : typle, array
            Offset of image from current center (might be issues if more than one value is non-zero)
        
        Returns
        -------
        image_data : array
            Acquired image
        '''
        if shape[0] > 512:
            binning = 4
        else:
            binning = 8
        
        myStemSearchParams = self.Acq.Detectors.AcqParams
        myStemSearchParams.Binning = binning
        myStemSearchParams.ImageSize = 1 # Size of image (0 = full size, 1 = half size, 2 = quarter size)
        myStemSearchParams.DwellTime = dwell
        self.Acq.Detectors.AcqParams = myStemSearchParams
        
        if self.TIA.AcquisitionManager().isAcquiring:
            self.TIA.AcquisitionManager().Stop()
        self.unblank()
        # Acquire an image
        acquiredImageSet = self.Acq.AcquireImages()
        with safearray_as_ndarray:
            image_data = acquiredImageSet(0).AsSafeArray # get data as ndarray
        self.blank()
        
        window1 = self.TIA.ActiveDisplayWindow()
        Im1 = window1.FindDisplay(window1.DisplayNames[0]) #returns an image display object
        unit1 = Im1.SpatialUnit #returns SpatialUnit object
        unitName = unit1.unitstring #returns a string (such as nm)
        calX = self.TIA.ScanningServer().ScanResolution
        calY = self.TIA.ScanningServer().ScanResolution

        return image_data, calX, calY, unitName

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
        #print('target enc =', target_enc)
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

class Zonexus_Server():
    def __init__(self, port, zncom='COM3'):
        
        self.microscope = TIA_control()
        self.zn_control = Zonexus_Continuous_Rotation_Stage_Control() # FOR TESTING
        
        context = zmq.Context()
        serverSocket = context.socket(zmq.REP)
        serverSocket.bind('tcp://*:'+str(port))
        print('Server Online')
        
        self.refImage = None

        while True:
            data = serverSocket.recv()
            self.d = pickle.loads(data)
            instruction = self.d['type']
            print(instruction)
            
            if instruction == 'ping':
                reply_message = 'pinged'
                reply_data = None
            elif instruction == 'get_zn_alpha':
                reply_message = 'alpha acquired'
                reply_data = self.zn_control.getAlpha()
                #reply_data = getAlphaDummy()
            elif instruction == 'set_zn_alpha':
                reply_message = 'target alpha reached'
                reply_data = self.zn_control.setAlpha(self.d['targetAlpha'])
                #reply_data = self.d['targetAlpha']
            elif instruction == 'ref':
                self.refImage, _, _, _ = self.microscope.microscope_acquire_image(self.d['dwell'], self.d['shape'])
                reply_message = 'reference image set'
                reply_data = self.refImage
            elif instruction == 'image':
                reply_message = 'image acquired'
                reply_data = self.microscope.microscope_acquire_image(self.d['dwell'], self.d['shape'], self.d['offset'])
            elif instruction == 'move_stage':
                reply_message = 'stage moved'
                reply_data = self.microscope.move_stage_delta(self.d['dX'], self.d['dY'], self.d['dZ'], self.d['dA'])
            elif instruction == 'set_mag':
                reply_message = 'mag changed'
                reply_data = self.microscope.set_mag(self.d['mag'])
            elif instruction == 'close_column_valve':
                reply_data = self.microscope.close_column_valve()
                if not self.microscope._microscope.Vacuum.ColumnValvesOpen:
                    reply_message = 'column valve closed'
                else:
                    reply_message = 'column valve NOT closed'
            else:
                reply_message = None
                reply_data = None
            
            reply_d = {'reply_message': reply_message,
                       'reply_data': reply_data}
            
            print(reply_d)
            
            serverSocket.send(pickle.dumps(reply_d))
    
if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--serverport', action='store', type=int, default=7001, help='server port')
    
    args = parser.parse_args()
    
    serverport = args.serverport
    zncom = 'COM3'
    
    server = Zonexus_Server(serverport, zncom)
