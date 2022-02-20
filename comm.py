import serial
import time

print("connecting...")
serialPort = serial.Serial(
    port="/dev/esp32", baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE
)
print("connected")

serialString = ""  # Used to hold data coming over UART
while 1:
    # Wait until there is data waiting in the serial buffer
    if serialPort.in_waiting > 0:

        # Read data out of the buffer until a carraige return / new line is found
        serialString = serialPort.readline()

        # Print the contents of the serial data
        try:
            print(serialString.decode("Ascii"))
        except:
            pass
