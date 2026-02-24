# -*- coding: utf-8 -*-
"""
Created on Mon Sep 30 11:24:34 2024

@author: alexa
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import zmq
import pickle

from matplotlib.backends.backend_qt5agg import FigureCanvas
from matplotlib.figure import Figure
from matplotlib.backend_bases import MouseButton

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

def cross_correlate(im0, im1):
    p0 = np.zeros((im0.shape[0], im0.shape[1]))
    p1 = np.zeros((im0.shape[0], im0.shape[1]))
    p1[p1.shape[0]//2-im1.shape[0]//2:p1.shape[0]//2-im1.shape[0]//2 + im1.shape[0],
       p1.shape[1]//2-im1.shape[1]//2:p1.shape[1]//2-im1.shape[1]//2 + im1.shape[1]] = im1
    f0 = np.fft.fft2(im0)
    f1 = np.fft.fft2(p1)
    f0 *= np.conj(f1)
    c = np.fft.ifft2(f0)
    return np.fft.fftshift(c.real)
        
def registration(refImage, curImage, pixelSize):
    corr = cross_correlate(curImage-curImage.mean(), refImage-refImage.mean())
    corr_arg = np.array(np.unravel_index(np.argmax(corr), corr.shape))
    offset = (corr_arg-np.array(refImage.shape)/2)
    offset_xy = offset*pixelSize
    #print(offset_xy)
    return offset_xy

class WorkerSignals(QObject):
    alpha = pyqtSignal(bytes)
    status = pyqtSignal(bytes)
    canvas = pyqtSignal(bytes)
    stopped = pyqtSignal(int)

class Worker(QRunnable):
    def __init__(self, fn, *args, **kwargs):
        super(Worker, self).__init__()
        
        # Store constructor arguments (re-used for processing)
        self.fn = fn
        self.args = args
        self.kwargs = kwargs
        self.signal = WorkerSignals()
        
        # Add the callback to our kwargs
        self.kwargs['alpha_callback'] = self.signal.alpha
        self.kwargs['status_callback'] = self.signal.status
        self.kwargs['canvas_callback'] = self.signal.canvas
        self.kwargs['stopped_callback'] = self.signal.stopped
    
    @pyqtSlot()
    def run(self):
        result = self.fn(*self.args, **self.kwargs)

class ZoNexus_Client():
    def __init__(self, host, port):
        
        self.status_callback = None
        self.canvas_callback = None
        self.alpha_callback = None
        self.stopped_callback = None
        
        self.pixelSize = None
        
        context = zmq.Context()
        self.ClientSocket = context.socket(zmq.REQ)
        self.ClientSocket.setsockopt(zmq.RCVTIMEO, 5000)  # 5 second receive timeout
        self.ClientSocket.setsockopt(zmq.SNDTIMEO, 5000)  # 5 second send timeout
        self.ClientSocket.connect(f"tcp://{host}:{port}")
        print(f'Connecting to ZoNexus server at {host}:{port}...')
        self.status_cb(f'Connecting to ZoNexus server at {host}:{port}...')

        response = self.send_traffic({'type': 'ping'})
        if response is not None and response.get('reply_data') == 'pinged':
            print('Connected to ZoNexus server!')
            self.status_cb('Connected to ZoNexus server!')
            
    def status_cb(self, message):
        if self.status_callback is not None:
            self.status_callback.emit(pickle.dumps(message))
            
    def canvas_cb(self, image):
        if self.canvas_callback is not None:
            self.canvas_callback.emit(pickle.dumps(image))
    
    def alpha_cb(self, alpha):
        if self.alpha_callback is not None:
            self.alpha_callback.emit(pickle.dumps(alpha))
    
    def set_callbacks(self, alpha_callback, status_callback, canvas_callback, stopped_callback):
        self.alpha_callback = alpha_callback
        self.status_callback = status_callback
        self.canvas_callback = canvas_callback
        self.stopped_callback = stopped_callback
    
    def send_traffic(self, message):
        '''
        Sends and receives messages from the server.
        
        Parameters
        ----------
        message : dict
            Message for the server.
        
        Returns
        -------
        response : dict
            Response from the server.
        '''
        print(f'Sending: {message}')
        try:
            self.ClientSocket.send(pickle.dumps(message))
            response = pickle.loads(self.ClientSocket.recv())
            print(f'Received: {response}')
            return response
        except zmq.Again:
            msg = 'ERROR: No response from server (timeout). Is the server running?'
            print(msg)
            self.status_cb(msg)
            return None

    def close_column_valve(self):
        d = {'type': 'close_column_valve'}
        Response = self.send_traffic(d)

    def acquire_image(self, dwell, size, mag, offset=(0,0), dwell_check=1e-3, alpha_callback=None, status_callback=None, canvas_callback=None, stopped_callback=None):
        self.set_callbacks(alpha_callback, status_callback, canvas_callback, stopped_callback)
        
        if dwell_check is not None:
            if dwell > dwell_check:
                dwell = dwell*1e-6
        
        print(dwell, size, mag, offset)
        
        self.set_mag(mag)
        
        shape = (size, size)
        d = {'type': 'image',
             'dwell': dwell,
             'shape': shape,
             'offset': offset}
        
        Response = self.send_traffic(d)
        
        image_data, calX, calY, unitName = Response['reply_data']
        self.pixelSize = calX
        self.canvas_cb(image_data)
        return image_data, calX
    
    def move_compustage(self, dX=0, dY=0, dZ=0, alpha_callback=None, status_callback=None, canvas_callback=None, stopped_callback=None):
        self.set_callbacks(alpha_callback, status_callback, canvas_callback, stopped_callback)
        d = {'type': 'move_stage',
             'dX': dX,
             'dY': dY,
             'dZ': dZ,
             'dA': 0}
        print(d)
        Response = self.send_traffic(d)
    
    def get_alpha_zn(self, alpha_callback=None, status_callback=None, canvas_callback=None, stopped_callback=None):
        self.set_callbacks(alpha_callback, status_callback, canvas_callback, stopped_callback)
        d = {'type': 'get_zn_alpha'}
        Response = self.send_traffic(d)
        alpha = Response['reply_data']
        print(f'alpha = {alpha:.3f}')
        self.alpha_cb(alpha)
        #return alpha
    
    def set_alpha_zn(self, A, alpha_callback=None, status_callback=None, canvas_callback=None, stopped_callback=None):
        self.set_callbacks(alpha_callback, status_callback, canvas_callback, stopped_callback)
        d = {'type': 'set_zn_alpha',
             'targetAlpha': A}
        Response = self.send_traffic(d)
        #return self.get_alpha_zn()
        
    def set_mag(self, mag):
        d = {'type': 'set_mag', 'mag': mag}
        Response = self.send_traffic(d)
    
    def move_on_click(self, x, y, alpha_callback=None, status_callback=None, canvas_callback=None, stopped_callback=None):
        self.set_callbacks(alpha_callback, status_callback, canvas_callback, stopped_callback)
        if self.pixelSize is not None:
            self.move_compustage(dX=float(x)*self.pixelSize, dY=float(y)*self.pixelSize) # X AND Y MAY NEED SWAPPING AND/OR INVERTING
            
    def centering(refImage, xymax=100e-9, ntries=4, df_range=None, cal_factor=1.0):
        NOT_CENTERED = True
        
        while NOT_CENTERED and ntries > 0:
            '''
            # NEEDS IMPLEMENTING
            if df_range is not None:
                focusing(df_range)
            '''
            curImage, pixelSize = acquire_image(search_dwell, search_size, search_mag)
            
            offset = registration(refImage, curImage, pixelSize) # Perform registration
            self.status_cb('offset = ', offset) # for debugging
            if abs(offset[0]) > xymax or abs(offset[1]) > xymax: # Move if needed
                ntries +=-1
                self.move_compustage(dX=offset[0]*cal_factor, dY=-offset[1]*cal_factor) # y may need positive or negative sign depending on eucentric height
                time.sleep(1)
            else:
                NOT_CENTERED = False
                self.status_cb('Centered')
        
        if NOT_CENTERED and ntries <= 0:
            raise ValueError('Number of attempts to center has exceeded ntries')
    
    def acquire_tilt_series(n_tilts, tilt_step,
                            search_dwell, search_size, search_mag,
                            acquire_dwell, acquire_size, acquire_mag,
                            close_valve_at_end=False,
                            cent_xymax=40e-9, cent_ntries=12, cent_cal_factor=1.1,
                            alpha_callback=None, status_callback=None, canvas_callback=None, stopped_callback=None):
        
        self.set_callbacks(alpha_callback, status_callback, canvas_callback, stopped_callback)
        
        self.status_cb('tilt_no = 0')
        
        self.tilt_series_images = np.zeros((n_angles, acquire_size, acquire_size))
        refImage, calX = acquire_image(acquire_dwell, acquire_size, acquire_mag) # take initial (also refeience) image
        self.tilt_series_images[0] = refImage
        
        for tilt_no in range(1, n_angles):

            self.status_cb(f'tilt_no = {tilt_no}')
            
            ''' Tilt by tilt_step '''
            dA = np.deg2rad(tilt_step)
            move_stage_delta(dA=dA)
            
            ''' Centering '''
            self.centering(refImage, xymax=cent_xymax, ntries=cent_ntries, cal_factor=cent_cal_factor)
                
            ''' Acquire HAADF image '''
            self.tilt_series_images[tilt_no], _ = acquire_image(acquire_dwell, acquire_size)
                
        if close_valve_at_end:
            self.close_column_valve()
        
class ZoNexus_GUI(QWidget):
    def __init__(self, host='localhost', port='7001', parent=None):
        super().__init__(parent)
        
        self.setWindowTitle('ZoNexus GUI')
        
        outerLayout = QHBoxLayout()
        
        imagePanelLayout = QVBoxLayout()
        
        self.im_size = 1024
        self.blank_image = np.zeros((self.im_size,self.im_size))
        
        self.fig, self.ax = plt.subplots(1,1,figsize=(10,10))
        self.ax.set_axis_off()
        self.ax.axis('equal') 
        self.imax = self.ax.matshow(self.blank_image)

        self.fig.set_tight_layout(True)
        self.canvas = FigureCanvas(self.fig)
        
        self.tracker = QLabel('Tracker:')
        
        imagePanelLayout.addWidget(self.canvas)
        imagePanelLayout.addWidget(self.tracker)
        
        self.search_dwell_input = QLineEdit('2')
        self.search_size_input = QLineEdit('256')
        self.search_mag_input = QLineEdit('20000')
        self.acquire_dwell_input = QLineEdit('3')
        self.acquire_size_input = QLineEdit('512')
        self.acquire_mag_input = QLineEdit('115000')
        self.n_tilts_input = QLineEdit('50')
        self.tilt_step_input = QLineEdit('1')

        settingsLayout = QFormLayout()
        settingsLayout.addRow('Search Dwell Time (us)', self.search_dwell_input)
        settingsLayout.addRow('Search Size', self.search_size_input)
        settingsLayout.addRow('Search Magnification', self.search_mag_input)
        settingsLayout.addRow('Acquire Dwell Time (us)', self.acquire_dwell_input)
        settingsLayout.addRow('Acquire Size', self.acquire_size_input)
        settingsLayout.addRow('Acquire Magnification', self.acquire_mag_input)
        
        settingsLayout.addRow('Number of tilts', self.n_tilts_input)
        settingsLayout.addRow('Tilt step (deg)', self.tilt_step_input)
        
        alphaButtonsLayout = QFormLayout()
        self.getAlphaButton = QPushButton('Get Current Alpha (deg)')
        self.getAlphaButton.clicked.connect(self.get_alpha_func)
        self.alphaLabel = QLabel('Target alpha = ')
        self.setTargetAlpha = QLineEdit('0')
        self.goToTargetAlphaButton = QPushButton('Go to Target Alpha')
        self.goToTargetAlphaButton.clicked.connect(self.set_alpha_func)
        self.acquireTiltSeriesButton = QPushButton('Acquire Tilt Series')
        self.acquireTiltSeriesButton.clicked.connect(self.acquire_tilt_series_func)
        self.stopTiltSeriesButton = QPushButton('Stop Tilt Series')
        self.stopTiltSeriesButton.clicked.connect(self.stop_tilt_series_func)
        
        alphaButtonsLayout.addWidget(self.getAlphaButton)
        alphaButtonsLayout.addWidget(self.alphaLabel)
        alphaButtonsLayout.addRow('Set Target Alpha (deg)', self.setTargetAlpha)
        alphaButtonsLayout.addWidget(self.goToTargetAlphaButton)
        alphaButtonsLayout.addWidget(self.acquireTiltSeriesButton)
        
        buttonsLayout = QGridLayout()
        self.search_button = QPushButton('Search')
        self.search_button.clicked.connect(self.search_func)
        self.acquire_button = QPushButton('Acquire')
        self.acquire_button.clicked.connect(self.acquire_func)
        
        buttonsLayout.addWidget(self.search_button, 0, 0)
        buttonsLayout.addWidget(self.acquire_button, 0, 1)
        
        statusLayout = QVBoxLayout()
        statusLayout.addWidget(QLabel('Status Box'))
        self.statusPanel = QTextEdit(readOnly=True)
        statusLayout.addWidget(self.statusPanel)
        
        rightSideLayout = QVBoxLayout()
        rightSideLayout.addLayout(settingsLayout)
        rightSideLayout.addLayout(alphaButtonsLayout)
        rightSideLayout.addLayout(buttonsLayout)
        rightSideLayout.addLayout(statusLayout)
        
        outerLayout.addLayout(imagePanelLayout)
        outerLayout.addLayout(rightSideLayout)
        
        self.setLayout(outerLayout)
        
        plt.connect('motion_notify_event', self.on_canvas_move)
        plt.connect('button_press_event', self.on_canvas_click)
        
        self.client = ZoNexus_Client(host, port)
    
    def worker_execute(self):
        self.thread_pool = QThreadPool()
        self.thread_pool.setMaxThreadCount(2)
        
        self.Worker.signal.alpha.connect(self.on_alpha_data_changed)
        self.Worker.signal.status.connect(self.on_status_data_changed)
        self.Worker.signal.canvas.connect(self.on_canvas_data_changed)
        self.Worker.signal.stopped.connect(self.on_stopped)
        self.thread_pool.start(self.Worker)
        
    def get_alpha_func(self, event):
        self.Worker = Worker(self.client.get_alpha_zn)
        self.worker_execute()
    
    def set_alpha_func(self, event):
        self.Worker = Worker(self.client.set_alpha_zn,
                             float(self.setTargetAlpha.text()))
        self.worker_execute()
    
    def acquire_tilt_series_func(self, event):
        n_tilts = int(self.n_tilts_input.text())
        tilt_step = float(self.tilt_step_input.text())
        
        search_dwell = float(self.search_dwell_input.text())*1e-6
        search_size = int(self.search_size_input.text())
        search_mag = int(self.search_mag_input.text())
        
        acquire_dwell = float(self.acquire_dwell_input.text())*1e-6
        acquire_size = int(self.acquire_size_input.text())
        acquire_mag = int(self.acquire_mag_input.text())
        
        self.Worker = Worker(self.client.acquire_tilt_series,
                             n_tilts, tilt_step,
                             search_dwell, search_size, search_mag,
                             acquire_dwell, acquire_size, acquire_mag,
                             close_valve_at_end=False)
        self.worker_execute()
    
    def stop_tilt_series_func(self, event):
        pass
    
    def search_func(self, event):
        dwell = float(self.search_dwell_input.text())*1e-6
        size = int(self.search_size_input.text())
        mag = int(self.search_mag_input.text())
        self.Worker = Worker(self.client.acquire_image, dwell, size, mag)
        self.worker_execute()
        
    def acquire_func(self, event):
        dwell = float(self.acquire_dwell_input.text())*1e-6
        size = int(self.acquire_size_input.text())
        mag = int(self.acquire_mag_input.text())
        self.Worker = Worker(self.client.acquire_image, dwell, size, mag)
        self.worker_execute()
        
    def on_canvas_click(self, event):
        if event.button is MouseButton.LEFT and event.inaxes:
            x, y = event.xdata, event.ydata
            if x > 0 and x < self.im_size and y > 0 and y < self.im_size:
                print(x,y)
                self.Worker = Worker(self.client.move_on_click,x,y)
                self.worker_execute()
        
    def on_canvas_move(self, event):
        if event.inaxes:
            x, y = event.xdata, event.ydata
            if x > 0 and x < self.im_size and y > 0 and y < self.im_size:
                x2 = x - self.im_size/2
                y2 = y - self.im_size/2
                if self.client.pixelSize is not None:
                    self.update_tracker(x2*self.client.pixelSize, y2*self.client.pixelSize)
            else:
                self.blank_tracker()
    
    @pyqtSlot(bytes) # connects to pyqtSignal object in receiver
    def on_alpha_data_changed(self, reply):
        '''
        Updates alpha.
        '''
        print('alpha updated')
        alpha = pickle.loads(reply)
        self.alphaLabel.setText(f'Target alpha = {alpha:.3f}')
    
    @pyqtSlot(bytes) # connects to pyqtSignal object in receiver
    def on_status_data_changed(self, reply):
        '''
        Updates status panel.
        '''
        print('status updated')
        message = pickle.loads(reply)
        self.statusPanel.append(str(message))
    
    @pyqtSlot(bytes) # connects to pyqtSignal object in receiver
    def on_canvas_data_changed(self, reply):
        '''
        Updates image.
        '''
        print('canvas updated')
        image = pickle.loads(reply)
        self.ax.axis('equal')
        self.imax.set_data(np.rot90(image))
        self.imax.set_clim(image.min(), image.max())
        self.canvas.draw()
    
    @pyqtSlot(int) # connects to pyqtSignal object in receiver
    def on_stopped(self, reply):
        '''
        Sets GUI after stop button pressed
        '''
        print('stopped')
        '''
        if reply == 1:
            self.stop_button.setEnabled(False)
            self.accept_button.setEnabled(True)
            self.reject_button.setEnabled(True)
            self.continue_button.setEnabled(True)
        '''
    
    def update_tracker(self, x, y):
        self.tracker.setText(f'Tracker: {x:.10f}, {y:.10f}') # FIX THIS
        
    def blank_tracker(self):
        self.tracker.setText(f'Tracker:')
        
if __name__ == "__main__":
    #host = '192.168.0.101'
    host = 'localhost'
    port = 7001
        
    app = QApplication(sys.argv)
    w = ZoNexus_GUI(host, port)
    w.show()
    sys.exit(app.exec_())