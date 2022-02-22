import serial
import time
import binascii
from PIL import Image 

print("connecting...")
serialPort = serial.Serial(
    port="/dev/esp32", baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE
)
print("connected")

image_data = ""  # Used to hold data the hexadecimal image data

s_counter = 0 
receiving_image = False 

while 1:
    if serialPort.in_waiting > 0:

        byte_in = serialPort.read()

        if receiving_image == True:
            image_data = image_data + byte_in.decode("utf-8")
        else:
            print(byte_in.decode("utf-8"), end ="")
        
        if byte_in.decode("utf-8") == 's':
            s_counter = s_counter+1

        else:
            s_counter = 0

        if s_counter == 5 and receiving_image == True:
            print("[Image Receiver]: stop receiving image")
            break

        elif s_counter == 5:
            print("\n[Image Receiver]: start receiving image")
            receiving_image=True

image_data = image_data[:-5] #remove the synchronization sequence

image_data=image_data.strip()
image_data=image_data.replace(' ', '')
image_data=image_data.replace('\n', '')
image_data = binascii.a2b_hex(image_data)
with open('image.jpg', 'wb') as image_file:
    image_file.write(image_data)
print("[Image Receiver]: image saved as: image.jpg")
img = Image.open('image.jpg')
img.show() 
