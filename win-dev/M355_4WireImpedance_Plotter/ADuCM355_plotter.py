import numpy as np
import pandas as pd
import pyqtgraph as pg
import time, datetime
import math
import logging
import smithplot as sp
import matplotlib.pyplot as plt

class ADuCM355_Plotter:

    def __init__(self, parent_ui):
        super().__init__()
        self.ui = parent_ui
        self.unique_freqs = None
        self.row_inds = list()
        self.x_time = list()
        self.tab1_plot_imag_vs_freq = False
        self.tab3_plot_s11 = True
        self.z = list()

    def change_tab1_plot(self):
        if self.tab1_plot_imag_vs_freq is True:
            self.tab1_plot_imag_vs_freq = False
            self.ui.togglePlotButton.setText("Toggle Mag/Ph Off")
        else:
            self.tab1_plot_imag_vs_freq = True
            self.ui.togglePlotButton.setText("Toggle Mag/Ph On")

    def change_tab3_plot(self):
        if self.tab3_plot_s11 is True:
            self.tab3_plot_s11 = False
            self.ui.toggleS11PlotButton.setText("Toggle Z'' vs Z' Off")
        else:
            self.tab3_plot_s11 = True
            self.ui.toggleS11PlotButton.setText("Toggle Z'' vs Z' On")

    def plot_tab1(self, df):
        if self.tab1_plot_imag_vs_freq is True:
            key1 = 'RX_REAL'
            key2 = 'RX_IMG'
            t1 = "Real Imp"
            t2 = "Imag Imp"
        else:
            key1 = 'MAG'
            key2 = 'PHASE'
            t1 = "Magnitude"
            t2 = "Phase"

        # get unique freqs from dataframe
        self.unique_freqs = df['FREQ_HZ'].unique()
        latest_data_inds = [df.loc[df['FREQ_HZ']==val].index.values[-1] for val in self.unique_freqs]
        if len(latest_data_inds) == 0:
            return
        try:
            xdata = df['FREQ_HZ'][latest_data_inds].values.tolist()
        except:
            return

        # get y-axis data
        ydata1 = df[key1][latest_data_inds].values.tolist()
        ydata2 = df[key2][latest_data_inds].values.tolist()
        abs_y = [abs(y1) for y1 in ydata1] + [abs(y2) for y2 in ydata2]

        # don't plot if there's no data
        if len(abs_y) <= 1:
            return

        # plot1 on page 1
        self.plot_fn(self.ui.plot1_tab1, xdata, ydata1, t1, 'Time')

        # plot2 on page 1
        self.plot_fn(self.ui.plot2_tab1, xdata, ydata2, t2, 'Time')

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
        self.plot_fn(self.ui.plot1_tab2, self.x_time, ydata, 'Real', 'Time')
    
    def plot_imag_vs_time(self, df):
        self.update_plot_inds(df)
        ydata = df['RX_IMG'][self.row_inds].values.tolist()
        self.plot_fn(self.ui.plot1_tab2, self.x_time, ydata, 'Imag', 'Time')

    def plot_avg_real_vs_time(self, df):
        self.update_plot_inds(df)
        N = self.ui.avgCountSpinbox.value()
        N = N if len(self.row_inds) >= N else len(self.row_inds)
        ydata = self.running_mean(df['RX_REAL'][self.row_inds].values,N).tolist()
        self.x_time = self.x_time[-len(ydata):]
        self.plot_fn(self.ui.plot1_tab2, self.x_time, ydata, 'Avg. Real', 'Time')

    def plot_avg_imag_vs_time(self, df):
        self.update_plot_inds(df)
        N = self.ui.avgCountSpinbox.value()
        N = N if len(self.row_inds) >= N else len(self.row_inds)
        ydata = self.running_mean(df['RX_IMG'][self.row_inds].values,N).tolist()
        self.x_time = self.x_time[-len(ydata):]
        self.plot_fn(self.ui.plot1_tab2, self.x_time, ydata, 'Avg. Imag', 'Time')

    def plot_mag_vs_time(self, df):
        self.update_plot_inds(df)
        ydata = df['MAG'][self.row_inds].values.tolist()
        self.plot_fn(self.ui.plot1_tab2, self.x_time, ydata, 'Magnitude', 'Time')
    
    def plot_phase_vs_time(self, df):
        self.update_plot_inds(df)
        ydata = df['PHASE'][self.row_inds].values.tolist()
        self.plot_fn(self.ui.plot1_tab2, self.x_time, ydata, 'Phase', 'Time')
    
    def update_s11_plot(self, df):
        # get unique freqs from dataframe
        self.unique_freqs = df['FREQ_HZ'].unique()
        latest_data_inds = [df.loc[df['FREQ_HZ']==val].index.values[-1] for val in self.unique_freqs]
        if len(latest_data_inds) == 0:
            return
        try:
            xdata = df['FREQ_HZ'][latest_data_inds].values.tolist()
        except:
            return

        # get phase and mag
        mag = df['MAG'][latest_data_inds].values.tolist()
        phase = df['PHASE'][latest_data_inds].values.tolist()

        # convert to real/imag impedance for smith chart
        real_z = np.asarray([m*math.cos(math.radians(ph)) for m,ph in zip(mag,phase)])
        imag_z = np.asarray([m*math.sin(math.radians(ph)) for m,ph in zip(mag,phase)])
        z = real_z + imag_z * 1j

        # update impedance Z0
        try:
            z0 = float(self.ui.z0_Edit.text())
        except:
            logging.warn('Invalid ZO; defaulting to 50')
            z0 = 50
        sp.SmithAxes.scDefaultParams['axes.impedance'] = z0

        # don't need to normalize, convert to list
        self.z = z.tolist()

        # s11 plot
        ph = self.ui.plot2_tab3.getPlotItem()
        if self.tab3_plot_s11 is True:
            ref_coef = (np.sqrt((real_z-z0)**2 + imag_z**2) / np.sqrt((real_z+z0)**2 + imag_z**2)).tolist()
            s11 = [20*math.log10(r) for r in ref_coef]
            self.ui.plot2_tab3.clear()
            self.ui.plot2_tab3.plot(y=s11, x=xdata,  pen=pg.mkPen('k', width=2))
            self.ui.plot2_tab3.autoRange()
            self.ui.plot2_tab3.setYRange(-60, 0)
            ph.setTitle('S11')
            ph.setLabel('bottom', 'S11 [dB]')
            ph.setLabel('left', 'Frequency [Hz]')
        else:
            self.ui.plot2_tab3.clear()
            #self.ui.plot2_tab3.setYRange(-2000, 2000)
            #self.ui.plot2_tab3.setXRange(-2000, 2000)
            self.ui.plot2_tab3.plot(y=imag_z.tolist(), x=real_z.tolist(),  pen=pg.mkPen('k', width=2))
            self.ui.plot2_tab3.autoRange(padding=0.05)
            ph.setTitle("Z'' vs Z'")
            ph.setLabel('bottom', "Z'")
            ph.setLabel('left', "Z''")
        
    def update_smith_chart(self, web_plot):
        # update impedance Z0
        try:
            z0 = float(self.ui.z0_Edit.text())
        except:
            logging.warn('Invalid ZO; defaulting to 100')
            z0 = 100
        #sp.SmithAxes.scDefaultParams['axes.impedance'] = z0
        #self.ui.smith_ax = self.ui.smithchart_tab3.add_subplot(111, projection='smith')
        if len(self.z) > 0:
            z_correct_factor = 100 / z0
            z_corrected = [zi*z_correct_factor for zi in self.z] 
            self.ui.smith_ax.clear()
            self.ui.smith_ax.plot(z_corrected, datatype=sp.SmithAxes.Z_PARAMETER)
            
            if web_plot is True:
                plt.ion()
                plt.draw()
                plt.show()
                time.sleep(0.001)

    def plot_fn(self,plot_handle,xdata,ydata,ylabel,xlabel):
        abs_y = [abs(y) for y in ydata]
        max_y_val = max(abs_y)
        if max_y_val == 0:
            ylim_log10 = 10
        else:
            try:
                ylim_log10 = 10**(int(math.log10(max_y_val))+1)
            except:
                return

        ph = plot_handle.getPlotItem()
        ph.setTitle(ylabel)
        ph.setLabel('bottom', xlabel)
        ph.setLabel('left', ylabel)

        plot_handle.clear()
        plot_handle.setYRange(-ylim_log10, ylim_log10)
        plot_handle.plot(y=ydata, x=xdata, pen=pg.mkPen('k', width=2))
