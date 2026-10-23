import sys
import serial
import pynmea2
from datetime import datetime
from pathlib import Path

# Ensure parent directory is in sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from data_parser.telemetry_manager import telemetry_manager

# Data sheet:
# https://content.u-blox.com/sites/default/files/MAX-M10S_DataSheet_UBX-20035208.pdf

# Config serial port according to hardware
SERIAL_PORT = "/dev/ttyUSB0"  # Replace with serial port

# Default m10 baud rate
BAUD_RATE = 9600


def parse_nmea_stream(port=SERIAL_PORT, baud=BAUD_RATE):
    try:
        ser = serial.Serial(port, baud, timeout=1.0)
    except serial.serialutil.SerialException as e:
        print(f"Error opening serial port {port}: {e}")
        return

    print(f"Connected to MAX-M10S on {port} @ {baud} baud")
    print(f"{'Time (UTC)':<12} | {'Latitude':<14} | {'Longitude':<14} "
          f"| {'Altitude':<12} | {'Sats':<5} | {'Fix Quality'}")
    print("-" * 75)

    try:
        while True:
            msg = ser.readline()
            if not msg:
                continue

            try:
                decoded_line = msg.decode('ascii', errors='replace').strip()

                if decoded_line.startswith('$'):
                    # Update global telemetry manager
                    telemetry_manager.parse_nmea_sentence(decoded_line)

                    parsed = pynmea2.parse(decoded_line)

                    if isinstance(parsed, pynmea2.GGA):
                        lat = f"{parsed.latitude:.6f}° {parsed.lat_dir}" if parsed.latitude else "No Fix"
                        lon = f"{parsed.longitude:.6f}° {parsed.lon_dir}" if parsed.longitude else "No Fix"
                        alt = f"{parsed.altitude} {parsed.altitude_units}" if parsed.altitude else "N/A"
                        sats = str(parsed.num_sats) if parsed.num_sats else "0"
                        fix_q = {
                            0: "No Fix",
                            1: "GPS Fix",
                            2: "DGPS Fix",
                            4: "RTK Fixed",
                            5: "RTK Float"
                        }.get(int(parsed.gps_qual) if parsed.gps_qual else 0, "Unknown")

                        timestamp = parsed.timestamp.strftime('%H:%M:%S') if parsed.timestamp else "--:--:--"

                        print(f"{timestamp:<12} | {lat:<14} | {lon:<14} | {alt:<12} | {sats:<5} | {fix_q}")

            except pynmea2.ParseError:
                continue
            except UnicodeDecodeError:
                continue

    except KeyboardInterrupt:
        ser.close()
        print(f"\nClosing serial port: {port}")


if __name__ == "__main__":
    target_port = sys.argv[1] if len(sys.argv) > 1 else SERIAL_PORT
    target_baud = int(sys.argv[2]) if len(sys.argv) > 2 else BAUD_RATE
    parse_nmea_stream(target_port, target_baud)
