

#RECIEVER
from machine import UART, Pin
import utime

# Initialize UART on GPIO 0 (TX) and GPIO 1 (RX)
uart1 = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

while True:
    if uart1.any():
        # Read and print the received message
        received_data = uart1.read()
        print("Received:", received_data.decode('ascii').strip())



#SENDER

from machine import UART, Pin
import utime
# Initialize UART on GPIO 0 (TX) and GPIO 1 (RX)
uart1 = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

count = 0 
while True:
    # Send a message
    count += 1
    uart1.write(f"RC: {count}\r\n")
    utime.sleep(0.01)

