import numpy as np
import pandas as pd
import pyqtgraph as pg
import time, datetime
import math

class ADuCM355_Plotter:

    def __init__(self, parent_ui):
        super().__init__()
        self.ui = parent_ui
        self.unique_freqs = None
        self.row_inds = list()
        self.x_time = list()

    def plot_real_imag_vs_freq(self, df):
        # get unique freqs from dataframe
        self.unique_freqs = df['FREQ_HZ'].unique()
        latest_data_inds = [df.loc[df['FREQ_HZ']==val].index.values[-1] for val in self.unique_freqs]
        xdata = df['FREQ_HZ'][latest_data_inds].values.tolist()

        # get y-axis data
        ydata1 = df['RX_REAL'][latest_data_inds].values.tolist()
        ydata2 = df['RX_IMG'][latest_data_inds].values.tolist()
        abs_y = [abs(y1) for y1 in ydata1] + [abs(y2) for y2 in ydata2]

        # don't plot if there's no data
        if len(abs_y) == 0:
            return

        max_y_val = max(abs_y)
        ylim_log10 = 10**(int(math.log10(max_y_val))+1)

        # real plot on page 1
        self.ui.plot1_tab1.clear()
        self.ui.plot1_tab1.setYRange(-ylim_log10, ylim_log10)
        self.ui.plot1_tab1.plot(y=ydata1, x=xdata)

        # image plot on page 1
        self.ui.plot2_tab1.clear()
        self.ui.plot2_tab1.setYRange(-ylim_log10, ylim_log10)
        self.ui.plot2_tab1.plot(y=ydata2, x=xdata)

    def running_mean(self, x, N):
        cumsum = np.cumsum(np.insert(x, 0, 0)) 
        return (cumsum[N:] - cumsum[:-N]) / float(N)

    def update_plot_inds(self, df):
        freq_ind = self.ui.freqIndexSpinbox.value()
        self.row_inds = df.index[df['FREQ_HZ'] == self.unique_freqs[freq_ind]].tolist()
        self.x_time = df['UTC_TIME'][self.row_inds].values.tolist()
        x0 = self.x_time[0]
        self.x_time[:] = [(x - x0  + 1) for x in self.x_time]

    def plot_real_vs_time(self, df):
        self.update_plot_inds(df)
        ydata = df['RX_REAL'][self.row_inds].values.tolist()
        self.plot1_tab2(self.x_time, ydata, 'Real', 'Time')
    
    def plot_imag_vs_time(self, df):
        self.update_plot_inds(df)
        ydata = df['RX_IMG'][self.row_inds].values.tolist()
        self.plot1_tab2(self.x_time, ydata, 'Imag', 'Time')

    def plot_avg_real_vs_time(self, df):
        self.update_plot_inds(df)
        N = self.ui.avgCountSpinbox.value()
        N = N if len(self.row_inds) >= N else len(self.row_inds)
        ydata = self.running_mean(df['RX_REAL'][self.row_inds].values,N).tolist()
        self.x_time = self.x_time[-len(ydata):]
        self.plot1_tab2(self.x_time, ydata, 'Avg. Real', 'Time')

    def plot_avg_imag_vs_time(self, df):
        self.update_plot_inds(df)
        N = self.ui.avgCountSpinbox.value()
        N = N if len(self.row_inds) >= N else len(self.row_inds)
        ydata = self.running_mean(df['RX_IMG'][self.row_inds].values,N).tolist()
        self.x_time = self.x_time[-len(ydata):]
        self.plot1_tab2(self.x_time, ydata, 'Avg. Imag', 'Time')

    def plot_mag_vs_time(self, df):
        self.update_plot_inds(df)
        ydata = df['MAG'][self.row_inds].values.tolist()
        self.plot1_tab2(self.x_time, ydata, 'Magnitude', 'Time')
    
    def plot_phase_vs_time(self, df):
        self.update_plot_inds(df)
        ydata = df['PHASE'][self.row_inds].values.tolist()
        self.plot1_tab2(self.x_time, ydata, 'Phase', 'Time')
    
    def plot1_tab2(self,xdata,ydata,ylabel,xlabel):
        abs_y = [abs(y) for y in ydata]
        max_y_val = max(abs_y)
        ylim_log10 = 10**(int(math.log10(max_y_val))+1)

        ph = self.ui.plot1_tab2.getPlotItem()
        ph.setTitle(ylabel)
        ph.setLabel('bottom', xlabel)
        ph.setLabel('left', ylabel)

        self.ui.plot1_tab2.clear()
        self.ui.plot1_tab2.setYRange(-ylim_log10, ylim_log10)
        self.ui.plot1_tab2.plot(y=ydata, x=xdata)
