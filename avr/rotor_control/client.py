import serial
import sys
import time
import msvcrt

ser = serial.Serial(port='COM4', baudrate=56000, timeout=1)
ser.flushInput()

def ds18b20_to_float(value):
    frac = 0.0

    if value & 0x08:
        frac += 0.5
    if value & 0x04:
        frac += 0.25
    if value & 0x02:
        frac += 0.125
    if value & 0x01:
        frac += 0.065

    return (value >> 4) + frac

def get_ds18b18_str(value):
    val_str = ''
    if value < int.from_bytes(b'\x7f00', byteorder='little', signed=True):
        val_str = '{:.2f}'.format( ds18b20_to_float(value))
    else:
        val_str = 'ERROR {}'.format(value & b'\x00ff')
    return val_str

def handle_key():
    key = int.from_bytes(msvcrt.getch(),byteorder='big')
    print('Key: {}'.format(key))
    time.sleep( 0.2 )
    key -= 48
    if key >= 0 and key < 4:
        # slaveId = 10, command = 0
        ser.write(b'\xaa\x00\xff\xa1' + bytes([key]))
    else:
        print('{} is not a command'.format(key))

print("CTRL + C to stop program!")
state = 0

while True:
    try:
        if msvcrt.kbhit():
            handle_key()

        time.sleep( 0.2 )
        # slaveId = 10, command = 0
        ser.write(b'\xaa\x00\xff\xa0\x00')
        if state == 0:
            byte = ser.read()
            if byte == b'\xaa':
                state = 1
            else:
                state = 0
                print('Error 1nd byte: {}'.format(byte))
        if state == 1:
            byte = ser.read()
            if byte == b'\x00':
                state = 2
            else:
                state = 0
                print('Error 2nd byte: {}'.format(byte))
        if state == 2:
            byte = ser.read()
            if byte == b'\xff':
                state = 3
            else:
                state = 0
                print('Error 3nd byte: {}'.format(byte))
        if state == 3:
            # Read full package
            ser_bytes = ser.read(17)

            # Convert byte stream to native values
            id = int.from_bytes(ser_bytes[0:1], byteorder='little', signed=False) 
            temp = int.from_bytes(ser_bytes[1:5], byteorder='little', signed=True) / 100 
            pres = int.from_bytes(ser_bytes[5:9], byteorder='little', signed=False) / 1000
            humi = int.from_bytes(ser_bytes[9:13], byteorder='little', signed=False) / 1024
            mtemp_raw = int.from_bytes(ser_bytes[13:15], byteorder='little', signed=True)
            mtemp = get_ds18b18_str(mtemp_raw)
            speed = int.from_bytes(ser_bytes[15:16], byteorder='little', signed=False)
            setpoint = int.from_bytes(ser_bytes[16:17], byteorder='little', signed=False)

            print('id: {0} Temp: {1:.2f} Pres: {2:.2f} Humi: {3:.2f} Motor Temp: {4} "{5:02x}" Speed: {6} SetPoint {7}'
                .format(id, temp, pres, humi, mtemp, mtemp_raw, speed, setpoint)
                )
            state = 0
    except KeyboardInterrupt:
        print('interrupted!')
        break