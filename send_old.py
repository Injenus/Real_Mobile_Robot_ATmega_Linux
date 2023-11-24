import serial
import time
import struct
from PyQt5 import QtWidgets, uic
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtCore import QIODevice
'''
PROTOCOL:  <#><Hash-Sum><byte>..<byte>
Hash-Sum is 8-bit number

'''
app = QtWidgets.QApplication([])
ui = uic.loadUi('gui_2.ui')
ui.setWindowTitle('Data Send')

is_run = True
send_timer = time.time()
#send_period = 0.1
init_sb = '#'
send_flag = False
receive_flag = False
receive_data_byte = bytearray(46)
rec_ind = 0

############################
type_move, val_move = 1, 15
arm_xyz_mode = [21, 10, 25, 1]
audio_mode = 5

def get_data_int_to_tx(type_move,val_move,x,y,z,mode,audio):
    return [type_move,val_move,x,y,z,mode,audio]




# ser = serial.Serial('/dev/ttyACM0', 115200)
###################
def refresh_serial_list():
    ui.serial_combobox.clear()
    port_list = []
    ports = QSerialPortInfo().availablePorts()
    for port in ports:
        port_list.append((port.portName()))
    ui.serial_combobox.addItems(port_list)

def open_port():
    serial.setPortName(ui.serial_combobox.currentText())
    serial.open(QIODevice.ReadWrite)
    print(ui.serial_combobox.currentText())
    ui.text_status_serial.setText(ui.serial_combobox.currentText()+' Open')

def close_port():
    serial.close()
    ui.text_status_serial.setText(ui.serial_combobox.currentText()+' Close')
    #ui.data_combobox.setEnabled(False)
    #ui.ready_send_data.setEnabled(False)

def send_data():
    global send_flag
    if (send_flag or 1): 
        data_text = [ui.lineEdit_0.text(), ui.lineEdit_1.text(), ui.lineEdit_2.text(), ui.lineEdit_3.text(), ui.lineEdit_4.text(), ui.lineEdit_5.text(), ui.lineEdit_6.text()]
        data_int = [int(el) for el in data_text]
        #byte_data = bytearray(struct.pack('BBHHHBB', *data_int[:2], *data_int[2:6], *data_int[6:])) # >= 0 !!!!!
        byte_data = bytearray(struct.pack('bbhhhbb', *data_int[:2], *data_int[2:6], *data_int[6:]))  
        send_pack = init_sb.encode('utf-8') + struct.pack('B',hash(byte_data)) + byte_data
        print('\nHash-sum is', send_pack[1])
        print('Sended',send_pack)
        serial.write(send_pack)
        send_flag = False

def send_test_data():
    global send_flag
    if (send_flag): 
        text = ui.lineEdit.text()
        b_data = bytearray.fromhex(text)
        send_pack = init_sb.encode('utf-8') + struct.pack('B',hash(b_data)) + b_data
        print('\nHash-sum is', send_pack[1])
        print('Sended',send_pack)
        send_flag = False
        serial.write(send_pack)

def print_rec_bytes():
    global receive_data_byte
    print('Curr res data:', receive_data_byte)

def receive_data():
    global send_flag, receive_flag, rec_ind
    rx_bytes = serial.readAll()
    #print(type(rx_bytes))
    ## convert to int
    print('Received: ',rx_bytes)
    #print("Len", len(rx_bytes))

    for byte in rx_bytes:
        try:
            if byte.decode() == '%':
                print('INIT R')
                receive_flag = True
                rec_ind = 0
                rx_bytes = rx_bytes[2:]
                try:
                    dec_num = struct.unpack('<' + 'h'*(4//2), rx_bytes)
                    print(dec_num)
                    pass
                except:
                    print('err_rx_dec 16!!!')
                    pass
        except:
            #print('err_dec')
            pass
    
    if receive_flag:
        for i, byte in enumerate(rx_bytes):
            try:
                receive_data_byte[rec_ind] = byte
                rec_ind += 1
                print(rec_ind, byte)
            except:
                print('errr',rec_ind, byte)
            if rec_ind == 47:
                receive_flag = False
                print_rec_bytes()
                break

    send_flag = True


def constrain(val, min_val=0, max_val=255):
    return min(max_val, max(min_val, val))

def hash(byte_data):
    #crc = struct.pack('B', 0)
    crc = 0 
    for byte in byte_data:
        crc = (crc << 3) & (0xff) | byte
        crc = (crc << 4) & (0xff) | byte
    return crc


# data = [type_move, val_move, arm_xyz_mode[0], arm_xyz_mode[1], arm_xyz_mode[2], arm_xyz_mode[3], audio_mode]
# byte_data = bytearray(struct.pack('BBHHHBB', *data[:2], *data[2:6], *data[6:]))
# send_pack = struct.pack('B',hash(byte_data)) + byte_data

# print(byte_data)
# print("Hash-sum: ", send_pack[0])
# print("Send packet: ", send_pack)

# # temp = [256, 42]
# # byte_t = bytearray(struct.pack('HB', *temp[:1], *temp[1:]))


#################################
ui.pause_data.setEnabled(0)
ui.stop_data.setEnabled(0)
ui.ready_send_data.setEnabled(0)
ui.text_status_serial.setReadOnly(1)
ui.text_status_data.setReadOnly(1)
ui.textEdit_status.setReadOnly(1)
# ui.text_status_data.setText("Write hex-bytes only!!!\nFor example:  00 0a 07 14 f1  (you should write 10 bytes)")
ui.text_status_data.setText("Write 7 integer numbers according to labdels:")
ui.text_status_serial.setText("Select Port and Open it to Send and Receive data")
#######
serial = QSerialPort()
serial.setBaudRate(1000000)
refresh_serial_list()
#
ui.refresh_serial.clicked.connect(refresh_serial_list)
ui.open_serial.clicked.connect(open_port)
ui.close_serial.clicked.connect(close_port)
#
ui.send_data.clicked.connect(send_data)
ui.send_test_data.clicked.connect(send_test_data)
#
serial.readyRead.connect(receive_data)
###


ui.show()
app.exec()

# while is_run:
#     if time.time() - send_timer > send_period:
#         send_timer = time.time()
#         # ser.write()
#         # print(packet)
# else:
#     ser.close()

