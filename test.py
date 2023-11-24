# declaring byte value
byte_val = b'\x01\x00'

# converting to int
# byteorder is big where MSB is at start
int_val = int.from_bytes(byte_val, "little", signed="True")

# printing int equivalent
print(int_val)


from PyQt5.QtCore import QBitArray

bits = QBitArray(4)
bits.setBit(0, True)
print(bits[0])