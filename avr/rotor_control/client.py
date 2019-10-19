import serial
ser = serial.Serial('COM4')
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

print("CTRL + C to stop program!")
while True:
    try:
        # Read full package
        ser_bytes = ser.read(17)

        # Convert byte stream to native values
        id = int.from_bytes(ser_bytes[0:1], byteorder='little', signed=False) 
        temp = int.from_bytes(ser_bytes[1:5], byteorder='little', signed=True) / 100 
        pres = int.from_bytes(ser_bytes[5:9], byteorder='little', signed=False)
        humi = int.from_bytes(ser_bytes[9:13], byteorder='little', signed=False) / 1024
        mtemp_raw = int.from_bytes(ser_bytes[13:15], byteorder='little', signed=True)
        mtemp = ds18b20_to_float(mtemp_raw)
        speed = int.from_bytes(ser_bytes[15:16], byteorder='little', signed=False)
        setpoint = int.from_bytes(ser_bytes[16:17], byteorder='little', signed=False)

        print('id: {0} Temp: {1} Pres: {2} Humi: {3} Motor Temp: {4} RAW: {5} Speed: {6} SetPoint {7}'
            .format(id, temp, pres, humi, mtemp, mtemp_raw, speed, setpoint)
            )
    except KeyboardInterrupt:
        print('interrupted!')
        break