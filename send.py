import serial
import time
import struct
from PyQt5 import QtWidgets, uic
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtCore import QIODevice
from PyQt5.QtCore import QBitArray
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
is_new_pack = False
receive_data_byte = bytearray()
# rec_16int = [-5 for i in range(27)] # 21 - int16; послдение 6 - из двух байтов (2 ИК. 4 концевика)
rec_16int = [-5 for i in range(22+2)] # 22 - int16; ик, концевики
rec_ind = 0
rec_data = [-3 for i in range(24)] #
rec_dict = {0: "left_wh",1: "right_wh",2: "mode_move",3:'x_arm',4:'y_arm', 5:'z_arm',
            6:'mode_arm', 7: 'ax', 8:'ay', 9:'az', 10:'gx', 11:'gy', 12:'gz',
            13:'ang_x', 14:'ang_y', 15:'ang_z', 16:'odo_l', 17:'odo_r',
            18:'lidar_angle', 19:'lidar_dist', 20:'sonar_1', 21:'sonar_2',
            22:'ir', 23:'end_sens'}

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

def get_int16(bytes):
    int16 = -2
    int16 = int.from_bytes(bytes,  "little", signed="True")
    return int16

def get_8bit_from_byte(raw_byte):
    bit = [-1 for i in range(8)]
    bit = [int((raw_byte >> i) & 1) for i in range(7,-1,-1)]

    # for i, b in enumerate(QBitArray(raw_byte)):
    #     bit[i] = 1 if b else 0

    return bit

def get_int8(byte):
    int8 = -2
    int8 = struct.unpack('b', byte)[0]
    return int8

def label_data(data):
    global rec_dict, rec_data
    # for i, el in enumerate(data):
    #     print(rec_dict.get(i, 'ZHOPA'), ':', el)
    for i, el in enumerate(rec_data):
        print(rec_dict.get(i, 'ZHOPA'), ':', el)


def parsing(byte_arr):
    global rec_data
    #print("PARS", len(byte_arr), byte_arr)
    for i in range(22):
        rec_data[i] = get_int16(receive_data_byte[i * 2:i * 2 + 2])
        #print(i, rec_data[i])
    rec_data[22] = get_int8(receive_data_byte[44])
    #print(rec_data[22])
    rec_data[23] = get_int8(receive_data_byte[45])
    return rec_data

def real_rec_data():
    global send_flag, is_new_pack, rec_ind, receive_data_byte, rec_data
    buff = serial.readAll()
    #print(buff, len(buff))
    if not is_new_pack:
        rec_ind = 0
        for byte in buff:
            try:
                if byte.decode() == '%':
                    is_new_pack = True
                    #print('TRUEEE')
            except:
                pass
            rec_ind += 1
        if is_new_pack:
            receive_data_byte = buff[rec_ind-32:]
    else:
        receive_data_byte += buff
    #print('Cur db: ',receive_data_byte, len(receive_data_byte), rec_ind)

    if len(receive_data_byte) >= 48:
        '''
        так просто непроверишь, поэтому мы сначала пытаемся декодирвоать,
        Если фаил само собой в утиль, если норм, то кодируем в байты и ищем хэш
        
        а хрен там, давай в тупую try       '''


        # #проверяем контрольную сумму!!!!
        # calc_hash_sum = hash(receive_data_byte[2:48])
        #
        #
        # if (calc_hash_sum == get_int8(receive_data_byte[1])):
        #     # мы всё ok, давай парсить
        #     receive_data_byte = receive_data_byte[2:48]
        #     ## ПАРСИМ
        #     rec_data = parsing(receive_data_byte)
        #     print('Received:', rec_data)
        #     ui.textEdit_status.setText(str(rec_data))
        # else:
        #     print('Fail!! (last Received):', rec_data)
        #     ui.textEdit_status.setText('Fail! last received: '+str(rec_data))

        try:
            receive_data_byte = receive_data_byte[2:48]
            #print('split_rec_d', receive_data_byte, len(receive_data_byte))
            rec_data = parsing(receive_data_byte)
            print('Received:', rec_data)
            ui.textEdit_status.setText(str(rec_data))
            label_data(rec_data)
        except:
            print('Fail!! (last Received):', rec_data)
            ui.textEdit_status.setText('Fail! last received: '+str(rec_data))

        is_new_pack = False
        send_flag = True




def receive_data():
    global send_flag, receive_flag, rec_ind, receive_data_byte, rec_16int
    rx_bytes = serial.readAll()
    #print(type(rx_bytes))
    ## convert to int
    #print('Received: ',rx_bytes)
    #print("Len", len(rx_bytes))

    for byte in rx_bytes:
        try:
            if byte.decode() == '%':
                #print('INIT R')
                receive_flag = True
                rec_ind = 0
                rx_bytes = rx_bytes[2:]
        except:
            #print('err_dec')
            pass
    
    if receive_flag:
        if rec_ind == 0:
            receive_data_byte = rx_bytes
            rec_ind += 1
            #print('ffffff',receive_data_byte)
        elif rec_ind == 1:
            # receive_data_byte += rx_bytes
            # receive_data_byte += rx_bytes[:-2] # отурбаем два последних байта
            receive_data_byte += rx_bytes
            #print('seccc',receive_data_byte, '_ len', len(receive_data_byte))
            for i in range(22):
                # rec_16int[i] = int.from_bytes(receive_data_byte[i*2:i*2+2], "little", signed="True")
                rec_16int[i] = get_int16(receive_data_byte[i*2:i*2+2])

            # ir_data = get_8bit_from_byte(receive_data_byte[42:43][0])
            # print(type(ir_data), ir_data)
            # for i in range(2):
            #     rec_16int.append(ir_data[i])
            # ends_data = get_8bit_from_byte(receive_data_byte[44:46])
            # for i in range(4):
            #     rec_16int.append(ends_data[i])

            rec_16int[22] = get_int8(receive_data_byte[44:45])
            rec_16int[23] = get_int8(receive_data_byte[45:46])


            print('Received:',rec_16int)
            ui.textEdit_status.setText(str(rec_16int))
            receive_flag = False
        
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
# serial.readyRead.connect(receive_data) real_rec_data
serial.readyRead.connect(real_rec_data)
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

