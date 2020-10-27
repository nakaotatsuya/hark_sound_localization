#!/usr/bin/env python

import rospy
from hark_msgs.msg import HarkWave
from jsk_hark_msgs.msg import HarkPower

import numpy as np
import matplotlib.pyplot as plt
from scipy.io import wavfile
from scipy.signal import fftconvolve
import IPython
import pyroomacoustics as pra

import glob
import scipy.io.wavfile as wav

import pyqtgraph as pg
from pyqtgraph.Qt import QtGui,QtCore


class WaveDataNode():
    def __init__(self):
        self.sub = rospy.Subscriber("/HarkWave", HarkWave, self._callback, queue_size=1)

        #QtGraph
        self.UPDATE_SECOND = 10
        
        self.app = QtGui.QApplication([])
        self.app.quitOnLastWindowClosed()
        #Window
        self.win = QtGui.QMainWindow()
        self.win.setWindowTitle("aaa")
        self.win.resize(750,400)
        self.centralwid = QtGui.QWidget()
        self.win.setCentralWidget(self.centralwid)

        #Layout
        self.lay = QtGui.QVBoxLayout()
        self.centralwid.setLayout(self.lay)

        self.plotwid1 = pg.PlotWidget(name="bbb")
        self.plotitem1 = self.plotwid1.getPlotItem()
        self.plotitem1.setXRange(-15,15)
        self.plotitem1.setYRange(-15,15)
        #self.specAxis1 = self.plotitem1.getAxis("bottom")
        #self.specAxis1.setLabel("Frequency[Hz]")
        #self.specAxis2 = self.plotitem1.getAxis("left")
        #self.specAxis2.setLabel("Amplitude")
        self.curve1 = self.plotitem1.plot()
        self.lay.addWidget(self.plotwid1)

        self.win.show()
        #update timer setting
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(self.UPDATE_SECOND)

        self.c = 343.    # speed of sound
        self.fs = 16000  # sampling frequency
        self.nfft = 256  # FFT size
        self.freq_range = [300, 3500]

        self.flag = True

        room_dim = np.r_[10.,10.]
        self.echo = pra.circular_2D_array(center=room_dim/2, M=8, phi0=0, radius=37.5e-3)

        self.doa = pra.doa.algorithms["MUSIC"](self.echo, self.fs, self.nfft, c=self.c, num_src=1)

        self.t_data = np.empty((8,0))
        self.t_data_len = 512 * 5

        self.spatial_resp = np.empty((360,))
        # self.initializePlot()
        # self.mainLoop()

        self.pub = rospy.Publisher("/HarkPower", HarkPower, queue_size=1)

    def update(self):
        #plotting param
        base = 1.
        height = 10.
        true_col = [0,0,0]

        if self.t_data.shape == (8,self.t_data_len):
            # for l in self.ax.get_lines():
            #     l.remove()
                
            phi_plt = self.doa.grid.azimuth
        
            c_phi_plt = np.r_[phi_plt, phi_plt[0]]
            c_dirty_img = np.r_[self.spatial_resp, self.spatial_resp[0]]
            #self.ax.plot(c_phi_plt, base + height * c_dirty_img, linewidth = 3,linestyle="-", label="spatial_spectrum")

            x = (base + c_dirty_img * height) * np.cos(c_phi_plt)
            y = (base + c_dirty_img * height) * np.sin(c_phi_plt)
            self.curve1.setData(x,y)
            
    def _callback(self, msg):
        data_list = np.array([])
        
        for i in range(len(msg.src)):
            data_list = np.append(data_list , msg.src[i].wavedata)

        data_list = np.reshape(data_list, (len(msg.src), -1))
        data_list /= 32768.0
        #print(data_list)
        
        pub_msg = HarkPower()
        print(pub_msg)
        pub_msg.header = msg.header
        pub_msg.count = pub_msg.header.seq
        pub_msg.directions = 360
        self.pub.publish(pub_msg)

        self.t_data = np.hstack((self.t_data, np.array(data_list)))
        self.t_data = self.t_data[:,-self.t_data_len:]
        print(self.t_data.shape)

        if self.t_data.shape == (8,self.t_data_len):
            Y = pra.transform.stft.analysis(self.t_data.T, self.nfft, self.nfft // 2)
            #print(Y.shape)
            Y = Y.transpose([2, 1, 0])

            self.doa.locate_sources(Y, freq_range=self.freq_range)

            #normalize
            min_val = self.doa.grid.values.min()
            max_val = self.doa.grid.values.max()
            self.spatial_resp = (self.doa.grid.values - min_val)/(max_val - min_val)
        
    def initializePlot(self):
        plt.ion()
        fig = plt.figure()
        self.ax = fig.add_subplot(111, projection="polar")

        plt.title("MUSIC")

    def mainLoop(self):
        while not rospy.is_shutdown():
            self.updatePlot()
            self.drawPlot()

    def updatePlot(self):
        #plotting param
        base = 1.
        height = 10.
        true_col = [0,0,0]

        if self.t_data.shape == (8,self.t_data_len):
            for l in self.ax.get_lines():
                l.remove()
            self.ax.plot(self.c_phi_plt, base + height * self.c_dirty_img, linewidth = 3,linestyle="-", label="spatial_spectrum")

    def drawPlot(self):
        plt.draw()
        plt.pause(0.01)

        # phi_plt = doa.grid.azimuth
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection="polar")
        # c_phi_plt = np.r_[phi_plt, phi_plt[0]]
        # c_dirty_img = np.r_[spatial_resp, spatial_resp[0]]
        # ax.plot(c_phi_plt, base + height * c_dirty_img, linewidth = 3,
        #        linestyle="-", label="spatial_spectrum")
        # plt.title("MUSIC")


if __name__ == "__main__":
    rospy.init_node("wavedata_node")
    WaveDataNode()
    rospy.spin()
    #try:
    #    QtGui.QApplication.instance().exec_()
    #except:
    #    print('terminate')

