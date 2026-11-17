
import serial
import time

PORT = "COM5"
BAUD_RATE = 9600

try:
    with serial.Serial(PORT, BAUD_RATE, timeout=2) as arduino:
        time.sleep(2)  # Allow Arduino to reset

        for command in ["L", "l", "R", "r", "C"]:
            arduino.write(command.encode())
            arduino.flush()

            response = arduino.readline().decode(
                "utf-8", errors="replace"
            ).strip()

            print(f"Sent {command}: {response or '(No response)'}")

except serial.SerialException as error:
    print("Serial error:", error)
