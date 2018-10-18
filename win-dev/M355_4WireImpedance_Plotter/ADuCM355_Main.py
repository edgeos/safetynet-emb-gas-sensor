import sys
sys.setrecursionlimit(50000)
import logging
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import pyqtSignal
from ADuCM355_eval_board_tool import MyDialog
from ADuCM355_cmd_protocol_defs import *
from ADuCM355_plotter import ADuCM355_Plotter
from utils import get_serial_ports, open_serial_port
from struct import pack, unpack
import numpy as np
import pandas as pd
import pyqtgraph as pg
import struct, serial, threading, serialstruct, binascii
import time, datetime
import csv, math


# Uncomment below for terminal log messages
# logging.basicConfig(level=logging.DEBUG, format=' %(asctime)s - %(name)s - %(levelname)s - %(message)s')

class QTextEditLogger(logging.Handler):
    def __init__(self, parent):
        super().__init__()
        self.widget = QtWidgets.QPlainTextEdit(parent)
        self.widget.setReadOnly(True)
        self.widget.setGeometry(QtCore.QRect(180, 760, 721, 111))

    def emit(self, record):
        msg = self.format(record)
        self.widget.appendPlainText(msg)


class AppWindow(QtWidgets.QDialog):

    # to be thread safe, the plotting is handled by signals
    update_plots_signal = pyqtSignal()

    def __init__(self):
        super().__init__()

        self.ui = MyDialog()
        self.ui.setupUi(self)
        
        # connect logger to text box
        logTextBox = QTextEditLogger(self)
        logTextBox.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
        logging.getLogger().addHandler(logTextBox)
        logging.getLogger().setLevel(logging.DEBUG)
        self.ui.logTextBox = logTextBox

        # Add plots
        # Enable antialiasing for prettier plots
        pg.setConfigOptions(antialias=True)

        # initial state is NOT connected
        self.connected = False
        self.save_data = False # to save CSV flag
        self.running = False
        #self.save_data_file = f'{datetime.datetime.now():%Y%m%d_%H%M%S}' + '_ImpedanceData.csv'
        self.save_data_file = datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + '_ImpedanceData.csv'

        # refresh com ports
        self.ser = None
        self.tx_ready = False
        self.tx_packet = None
        self.tx_success = False
        self.ack_received = False
        self.refresh_com_ports()
        
        # connect buttons to tasks
        self.ui.connectButton.clicked.connect(self.connect_serial)
        self.ui.refreshButton.clicked.connect(self.refresh_com_ports)
        self.ui.runContButton.clicked.connect(self.start_measurements)
        self.ui.stopRunButton.clicked.connect(self.stop_measurements)
        self.ui.stopRunButton.setEnabled(False)
        self.ui.freqIndexSpinbox.setMinimum(0)
        self.ui.freqIndexSpinbox.setMaximum(0)
        self.ui.freqIndexSpinbox.valueChanged.connect(self.update_freq_text)
        self.ui.avgCountSpinbox.setMinimum(1)

        # register packet handlers
        self.packet_handlers = {
            CMD_PING[0]: self.handle_ping_cmd,
            CMD_START_MEASURE[0]: self.handle_start_measure,
            CMD_STOP_MEASURE[0]: self.handle_stop_measure,
            CMD_SEND_DATA[0]: self.handle_measurement_data,
            CMD_ACK[0]: self.handle_ack
        }

        self.plotter = ADuCM355_Plotter(self.ui)
        self.tab2_plot_handlers = {
            'Raw Real': self.plotter.plot_real_vs_time,
            'Raw Imag': self.plotter.plot_imag_vs_time,
            'Avg. Real': self.plotter.plot_avg_real_vs_time,
            'Avg. Imag': self.plotter.plot_avg_imag_vs_time,
            'Magnitude': self.plotter.plot_mag_vs_time,
            'Phase': self.plotter.plot_phase_vs_time
        }
        for key in self.tab2_plot_handlers.keys():
            self.ui.ringBox.addItem(key) 

        # start thread
        self.MUTEX_PROTECT_UART = False
        self.ser_thread = threading.Thread(target=self.read_from_serial_port)
        self.ser_thread.start()
        self.update_plots_signal.connect(self.update_plots)
        self.MUTEX_PROTECT_PLOTTER = False

        self.serialBuffer = bytearray()
        self.frameBuffer = bytearray()

        # for data
        self.current_timestamp = None
        self.df_cols = list(['UTC_TIME', 'FREQ_HZ', 'RCAL_REAL', 'RCAL_IMG', 'RX_REAL', 'RX_IMG', 'MAG', 'PHASE'])
        self.df = pd.DataFrame(columns=self.df_cols,dtype=np.float32)
        self.unique_freqs = list()

        self.show()

    def update_freq_text(self):
        freq_ind = self.ui.freqIndexSpinbox.value()
        self.ui.freqValueText.setText(str(self.unique_freqs[freq_ind]/1000))

    def update_plots(self):
        # prevent the signal from being emitted too quickly
        self.MUTEX_PROTECT_PLOTTER = True
        
        # get unique freqs from dataframe
        unique_freqs = self.df['FREQ_HZ'].unique().tolist()
        freq_matches = set(unique_freqs).intersection(self.unique_freqs)
        if len(freq_matches) != len(unique_freqs):
            self.unique_freqs = unique_freqs
            self.ui.freqIndexSpinbox.setMaximum(len(unique_freqs)-1)
            self.update_freq_text()
        
        
        self.plotter.plot_real_imag_vs_freq(self.df)
        if self.df.empty is not True:    
            tab2_plot_function = self.tab2_plot_handlers.get(self.ui.ringBox.currentText(), None)
            tab2_plot_function(self.df)
        
        self.MUTEX_PROTECT_PLOTTER = False

    def save_data_to_csv(self, data):
        with open(self.save_data_file,'a',newline='') as f:
            writer = csv.writer(f) 
            writer.writerow(data)

    def save_data_to_dataframe(self, data):
        # don't touch the data until plotting is done
        while self.MUTEX_PROTECT_PLOTTER is True:
            time.sleep(0.00001)

        self.df.loc[len(self.df.index)] = data
        
        # send signal when plotter is available
        self.update_plots_signal.emit()

    def start_measurements(self):
        if not self.connected:
            logging.warn("Not connected; can't start measurements!")
            return

        # update save data file 
        #self.save_data_file = f'{datetime.datetime.now():%Y%m%d_%H%M%S}' + '_ImpedanceData.csv'
        self.save_data_file = datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + '_ImpedanceData.csv'

        # check if we're saving data
        self.save_data = False
        if self.ui.checkBox.isChecked():
            self.save_data = True

            # write titles
            with open(self.save_data_file,'a',newline='') as f:
                writer = csv.writer(f) 
                writer.writerow(self.df_cols)

            logging.info('Logging all data to file: %s', self.save_data_file)
        
        # start collecting data
        self.send_start_measure()
        if self.tx_success is True:
            logging.info('Started continuous measurement...')
            self.ui.stopRunButton.setEnabled(True)
            self.ui.runContButton.setEnabled(False)
            self.running = True
            self.unique_freqs = list()

            # disable checking while running
            self.ui.checkBox.setDisabled(True)
        else:      
            logging.error('No ACK received to Start Measurement command!')
      
    def stop_measurements(self):
        # re-enable checkbox
        self.ui.checkBox.setDisabled(False)
        self.running = False

        # start collecting data
        self.send_stop_measure()
        if self.tx_success is True:
            logging.info('Stopped continuous measurement...')
            self.ui.stopRunButton.setEnabled(False)
            self.ui.runContButton.setEnabled(True)

            # don't touch the data until plotting is done
            while self.MUTEX_PROTECT_PLOTTER is True:
                time.sleep(0.00001)
            
            self.MUTEX_PROTECT_PLOTTER = True
            self.df = pd.DataFrame(columns=self.df_cols)
            self.MUTEX_PROTECT_PLOTTER = False
        else:      
            logging.error('No ACK received to Stop Measurement command!')

    def refresh_com_ports(self):
        # clear current ports
        self.ui.comPortEdit.clear()

        # populate the available COM ports list
        avail_ports = get_serial_ports()
        if len(avail_ports):
            logging.info('Scanned for ports, found %d available port(s)', len(avail_ports))
            for port in avail_ports:
                self.ui.comPortEdit.addItem(port)  
        else:
            logging.info('Scanned for ports, found none')
            self.ui.comPortEdit.addItem("No Ports!")

    def connect_serial(self):
        # disconnect if we're already connected
        if not self.connected:
            port = str(self.ui.comPortEdit.currentText())
            try:
                self.ser = open_serial_port(serial_port=port, baudrate=115200, timeout=0, write_timeout=0)
                logging.info('Opened port at %s', port)
            except (OSError, serial.SerialException):
                logging.warn('Unable to open serial port to %s', port)
            self.connected = True

            # send ping cmd and wait for ack
            self.ping_received = False
            self.send_ping()
            if self.tx_success is True:
                logging.info('Found ADuCM355 Impedance Measurement board!')
                
                self.ui.connectButton.setText("Disconnect")
                self.ui.refreshButton.setEnabled(False)
                self.ui.comPortEdit.setEnabled(False)

                # stop measurements if they happen to be going; if app crashed previously during a collect
                self.stop_measurements()
            else:
                logging.warn('No ADuCM355 at this port!')
                self.connected = False
                self.ser.close()
        else:
            # stop measure mode if it's currently enabled
            if self.running:
                self.stop_measurements()

            self.connected = False
            self.ser.close()
            logging.info('Closed serial port connection')

            # change button back
            self.ui.connectButton.setText("Connect")
            self.ui.refreshButton.setEnabled(True)
            self.ui.comPortEdit.setEnabled(True)

    def read_from_serial_port(self):
        while True:
            if self.connected and not self.MUTEX_PROTECT_UART:
                rx = self.ser.read(100,)
                if rx:
                    self.MUTEX_PROTECT_UART = True
                    self.serialBuffer.extend(rx)
                    self.handle_serial_buffer()
                    self.MUTEX_PROTECT_UART = False
                if self.tx_ready is True:
                    self.MUTEX_PROTECT_UART = True
                    self.ser.write(self.tx_packet)
                    self.tx_ready = False
                    self.MUTEX_PROTECT_UART = False
            else:
                time.sleep(0.1)
    
    def handle_serial_buffer(self):
        # search for start bytes
        st_ind = self.serialBuffer.find(START_BYTES)
        while st_ind is not -1:
            # check for stop byte
            stop_ind = st_ind+PACKET_LENGTH-1
            if (stop_ind+1) > len(self.serialBuffer):
                break
            
            if self.serialBuffer[stop_ind] is STOP_BYTE[0]:   
                self.frameBuffer = self.serialBuffer[st_ind:(stop_ind+1)]
                self.serialBuffer = self.serialBuffer[:st_ind] + self.serialBuffer[(stop_ind+1):]

                # parse and add to plottable data
                self.handle_frame_data()

                # look for next packet
                st_ind = self.serialBuffer.find(START_BYTES)
            else:
                # look for next packet
                st_ind = self.serialBuffer.find(START_BYTES, st_ind+1)
    
    def handle_frame_data(self):
        # decode packet
        st_bytes, cmd_byte, len_byte, payload_bytes, crc_bytes, _ = unpack(PACKET_FMT, self.frameBuffer)
        crc_data = st_bytes + bytearray([cmd_byte]) + bytearray([len_byte]) + payload_bytes

        # check CRC, discard o/w
        calc_crc = binascii.crc_hqx(crc_data, 65535)
        if calc_crc == crc_bytes:
            packet_handler_func = self.packet_handlers.get(cmd_byte, None)
            if packet_handler_func is None:
                logging.warn('Invalid CMD byte received: %d', cmd_byte)

            # send ACK immediately
            #self.send_ack()

            # handle payload
            packet_handler_func(payload_bytes)
        else:
            logging.warn('Received packet with invalid CRC')
        
        # clear frame buffer
        self.frameBuffer = None

    def handle_ping_cmd(self, payload):
        return

    def handle_start_measure(self, payload):
        return

    def handle_stop_measure(self, payload):
        return

    def handle_measurement_data(self, payload):
        _, _, imp_data_packed = unpack(DATA_PAYLOAD_FMT, payload) #decode

        # update time index if it's a new measurement collect
        self.current_timestamp = time.time()

        # add timestamp
        data = list([self.current_timestamp]) + list(unpack(IMPEDANCE_FMT, imp_data_packed)) # utctime, freq, dft_results[4], mag, phase

        # save to csv
        if self.save_data:
            self.save_data_to_csv(data)
        
        # update the data in memory; signal will be sent to update plots here
        self.save_data_to_dataframe(data)

    def handle_ack(self, payload):
        self.ack_received = True

    def send_start_measure(self):
        try:
            start_freq_text = float(self.ui.startFreqEdit.toPlainText())*1000 # convert from kHz to Hz
            step_freq_text = float(self.ui.stepFreqEdit.toPlainText())*1000
            stop_freq_text = float(self.ui.stopFreqEdit.toPlainText())*1000
            payload = pack(ENDIAN_FMT+FREQ_START_FMT,start_freq_text) + pack(ENDIAN_FMT+FREQ_STEP_FMT,step_freq_text) + pack(ENDIAN_FMT+FREQ_STOP_FMT,stop_freq_text) 
            measure_payload = payload + bytearray(MEASURE_PADDING_LENGTH)
        except:
            logging.error('Invalid text format for one or more arguments.')
            return

        freq_min = 0.1
        freq_max = 200000

        # check for valid entries
        error_found = False
        if start_freq_text < freq_min or start_freq_text > freq_max:
            error_found = True
            logging.error('Invalid entry for START FREQUENCY')
        if step_freq_text < freq_min or step_freq_text > freq_max:
            error_found = True
            logging.error('Invalid entry for STEP FREQUENCY')
        if stop_freq_text < freq_min or stop_freq_text > freq_max:
            error_found = True
            logging.error('Invalid entry for STOP FREQUENCY')
        if stop_freq_text < start_freq_text:
            error_found = True
            logging.error('Invalid entry, START FREQUENCY must be less than STOP FREQUENCY')
        if error_found is True:
            return

        packet = START_BYTES + CMD_START_MEASURE + bytearray([DATA_PAYLOAD_LENGTH]) + measure_payload
        calc_crc = pack(ENDIAN_FMT+CRC_FMT,binascii.crc_hqx(packet, 65535))
        packet += calc_crc + STOP_BYTE

        self.send_packet(packet, True)
        
    def send_stop_measure(self): 
        packet = START_BYTES + CMD_STOP_MEASURE + bytearray([DATA_PAYLOAD_LENGTH]) + bytearray(DATA_PAYLOAD_LENGTH)
        calc_crc = pack(ENDIAN_FMT+CRC_FMT,binascii.crc_hqx(packet, 65535))
        packet += calc_crc + STOP_BYTE

        self.send_packet(packet, True)   

    def send_ack(self):
        packet = START_BYTES + CMD_ACK + bytearray([DATA_PAYLOAD_LENGTH]) + bytearray(DATA_PAYLOAD_LENGTH)
        calc_crc = pack(ENDIAN_FMT+CRC_FMT,binascii.crc_hqx(packet, 65535))
        packet += calc_crc + STOP_BYTE

        self.send_packet(packet, False)
    
    def send_ping(self):
        packet = START_BYTES + CMD_PING + bytearray([DATA_PAYLOAD_LENGTH]) + bytearray(DATA_PAYLOAD_LENGTH)
        calc_crc = pack(ENDIAN_FMT+CRC_FMT,binascii.crc_hqx(packet, 65535))
        packet += calc_crc + STOP_BYTE
        
        self.send_packet(packet, True)
    
    def send_packet(self,packet,wait_for_ack):
        delay_sec = 0.05
        timeout_sec = 5
        max_delay_ct = timeout_sec / delay_sec
        self.tx_success = True
        if wait_for_ack:
            self.ack_received = False
        self.tx_packet = packet
        self.tx_ready = True

        if wait_for_ack:
            delay_ct = 0
            while self.ack_received is not True and delay_ct < max_delay_ct:
                time.sleep(delay_sec)
                delay_ct += 1
            if not self.ack_received:
                self.tx_success = False     

app = QtWidgets.QApplication(sys.argv)
w = AppWindow()
w.setFixedSize(w.size())
w.show()
sys.exit(app.exec_())