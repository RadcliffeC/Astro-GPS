import serial
import pynmea2
from datetime import datetime

#Data sheet
# https://content.u-blox.com/sites/default/files/MAX-M10S_DataSheet_UBX-20035208.pdf

# Config serial port according to hardware
SERIAL_PORT = "/dev/ttyUSB0" # Replace with serial port

# Default m10 baud rate
BAUD_RATE = 9600

def parse_nmea_stream():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1.0)
    except serial.serialutil.SerialException as e:
        print(f"Error opening serial port: {e}")
        return

    print(f"{'Time (UTC)':<12} | {'Latitude':<12} | {'Longitude':<12} "
          f"| {'Altitude':<12} | {'Sats':<5} | {'Fix Quality'}")

    print(f"-" *65)

    try:
        while True:
            msg = ser.readline()
            if not msg:
                continue

            try:
                decoded_line = msg.decode('ascii', errors='replace').strip()

                if decoded_line.startswith('$'):
                    msg = pynmea2.parse(decoded_line)

                    if isinstance(msg, pynmea2.GGA):
                        lat = f"{msg.latitude:.6f}° {msg.lat_dir}" if msg.latitude else "No Fix"
                        lon = f"{msg.longitude:.6f}° {msg.lon_dir}" if msg.longitude else "No Fix"
                        alt = f"{msg.altitude} {msg.altitude_units}" if msg.altitude else "N/A"
                        sats = msg.num_sats if msg.num_sats else "0"
                        fix_q = {
                            0: "No Fix",
                            1: "GPS Fix",
                            2: "DGPS Fix",
                            4: "RTK Fixed",
                            5: "RTK Float"
                        }.get(int(msg.gps_qual) if msg.gps_qual else 0, "Unknown")

                        timestamp = msg.timestamp.strftime('%H:%M:%S') if msg.timestamp else "--:--:--"

                        print(f"{timestamp:<12} | {lat:<12} | {lon:<13} | {alt:<8} | {sats:<5} | {fix_q}")

            except pynmea2.ParseError:
                continue
            except UnicodeDecodeError:
                continue

    except KeyboardInterrupt:
        ser.close()
        print(f"\nClosing serial port: {SERIAL_PORT}")


if __name__ == "__main__":
    parse_nmea_stream()