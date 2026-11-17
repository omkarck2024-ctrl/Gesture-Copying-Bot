
import serial
import time

# Replace COM3 with your actual Arduino port
PORT = "COM5"
BAUD_RATE = 9600

try:
    arduino = serial.Serial(PORT, BAUD_RATE, timeout=2)
    time.sleep(2)  # Arduino may reset when serial opens

    print("Connected to Arduino!")
    print("Sending a harmless test message...")

    arduino.write(b"TEST\n")
    time.sleep(1)

    response = arduino.readline().decode(
        "utf-8", errors="replace"
    ).strip()

    print("Arduino response:", response if response else "(No response)")
    arduino.close()

except serial.SerialException as error:
    print("Serial connection error:", error)
