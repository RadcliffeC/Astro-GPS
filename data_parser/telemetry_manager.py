import math
import time
import asyncio
import threading
import serial
import serial.tools.list_ports
import pynmea2
from datetime import datetime, timezone
from typing import Dict, Any, List, Optional

def haversine_distance_and_bearing(lat1: float, lon1: float, lat2: float, lon2: float):
    """
    Computes ground distance (meters) and bearing (degrees) between two lat/lon coordinates.
    """
    R = 6371000.0  # Earth radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2.0) ** 2 + \
        math.cos(phi1) * math.cos(phi2) * (math.sin(delta_lambda / 2.0) ** 2)
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    distance = R * c

    y = math.sin(delta_lambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - \
        math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
    bearing = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

    return distance, bearing

def calculate_slant_range_and_elevation(ground_distance: float, alt_rel: float):
    """
    Calculates slant range (direct 3D line-of-sight distance) and elevation angle (degrees) from ground station.
    """
    slant_range = math.sqrt(ground_distance ** 2 + alt_rel ** 2)
    if slant_range > 0:
        elevation = math.degrees(math.atan2(alt_rel, ground_distance))
    else:
        elevation = 0.0
    return slant_range, elevation


class TelemetryManager:
    def __init__(self):
        self.lock = threading.RLock()
        
        # Connection settings
        self.serial_port = "/dev/ttyUSB0"
        self.baud_rate = 9600
        self.serial_handle: Optional[serial.Serial] = None
        self.is_connected = False
        self.serial_running = False
        self.reader_thread: Optional[threading.Thread] = None

        # Simulation settings
        self.is_simulating = False
        self.sim_thread: Optional[threading.Thread] = None
        self.sim_time = 0.0

        # Launch pad coordinates (defaults to Spaceport America / White Sands or first 3D fix)
        self.pad_lat: Optional[float] = 32.9904
        self.pad_lon: Optional[float] = -106.9750
        self.pad_alt: Optional[float] = 1400.0  # MSL in meters
        self.pad_locked = False

        # Mission timing
        self.mission_start_time: Optional[float] = None
        self.is_mission_active = False

        # Current rocket state
        self.state: Dict[str, Any] = self._get_default_state()

        # Telemetry history for charts & map path (last 1000 points)
        self.history: List[Dict[str, Any]] = []
        self.max_history = 1500

        # Events log
        self.events: List[Dict[str, Any]] = []

        # Raw NMEA log buffer (last 50 lines)
        self.raw_nmea_logs: List[Dict[str, Any]] = []
        self.max_raw_logs = 100

        # Subscribers (WebSocket queues or async callbacks)
        self.subscribers: List[asyncio.Queue] = []
        self.loop: Optional[asyncio.AbstractEventLoop] = None

        # Velocity smoothing helpers
        self._last_time = time.time()
        self._last_alt = 0.0
        self._last_lat = 0.0
        self._last_lon = 0.0
        self._max_alt_msl = 0.0
        self._max_alt_agl = 0.0
        self._max_velocity = 0.0
        self._max_vvi = 0.0

        # Loss of signal tracker
        self.last_packet_time = 0.0
        self.packet_count = 0
        self.packet_rate = 0.0
        self._packet_timestamps = []

    def _get_default_state(self) -> Dict[str, Any]:
        return {
            "timestamp_utc": datetime.now(timezone.utc).strftime("%H:%M:%S.%f")[:-3],
            "met_seconds": 0.0,
            "met_formatted": "T+00:00:00.0",
            "connected": False,
            "simulating": False,
            "fix_quality": "No Fix",
            "fix_quality_code": 0,
            "has_fix": False,
            "satellites": 0,
            "hdop": 99.9,
            "vdop": 99.9,
            "latitude": self.pad_lat if self.pad_lat else 0.0,
            "longitude": self.pad_lon if self.pad_lon else 0.0,
            "altitude_msl": self.pad_alt if self.pad_alt else 0.0,
            "altitude_agl": 0.0,
            "max_altitude_msl": 0.0,
            "max_altitude_agl": 0.0,
            "vertical_speed": 0.0,  # m/s (climb rate)
            "ground_speed": 0.0,    # m/s
            "total_velocity": 0.0,  # m/s
            "max_velocity": 0.0,
            "heading": 0.0,         # deg
            "pad_distance": 0.0,    # m
            "pad_bearing": 0.0,     # deg
            "pad_slant_range": 0.0, # m
            "pad_elevation": 0.0,   # deg
            "flight_phase": "PAD",  # PAD, BOOST, COAST, APOGEE, DROGUE, MAIN, LANDED
            "signal_status": "DISCONNECTED", # LIVE, SIMULATION, LOS, DISCONNECTED
            "packet_rate": 0.0,
            "last_packet_age": 0.0,
            "pad_lat": self.pad_lat,
            "pad_lon": self.pad_lon,
            "pad_alt": self.pad_alt,
        }

    def set_event_loop(self, loop: asyncio.AbstractEventLoop):
        self.loop = loop

    def add_subscriber(self, queue: asyncio.Queue):
        with self.lock:
            self.subscribers.append(queue)

    def remove_subscriber(self, queue: asyncio.Queue):
        with self.lock:
            if queue in self.subscribers:
                self.subscribers.remove(queue)

    def broadcast_state(self, state: Dict[str, Any]):
        if not self.loop:
            return
        with self.lock:
            subscribers_copy = list(self.subscribers)
        
        for q in subscribers_copy:
            try:
                self.loop.call_soon_threadsafe(q.put_nowait, state)
            except Exception:
                pass

    def add_event(self, event_type: str, description: str, data: Optional[Dict[str, Any]] = None):
        event = {
            "timestamp": datetime.now(timezone.utc).strftime("%H:%M:%S.%f")[:-3],
            "met": self.state.get("met_formatted", "T+00:00:00.0"),
            "type": event_type,
            "description": description,
            "data": data or {}
        }
        with self.lock:
            self.events.append(event)
            if len(self.events) > 200:
                self.events.pop(0)

    def add_raw_nmea(self, sentence: str):
        log_entry = {
            "time": datetime.now(timezone.utc).strftime("%H:%M:%S.%f")[:-3],
            "raw": sentence
        }
        with self.lock:
            self.raw_nmea_logs.append(log_entry)
            if len(self.raw_nmea_logs) > self.max_raw_logs:
                self.raw_nmea_logs.pop(0)

    def set_pad_coordinates(self, lat: float, lon: float, alt: float = 0.0):
        with self.lock:
            self.pad_lat = lat
            self.pad_lon = lon
            self.pad_alt = alt
            self.pad_locked = True
            self.state["pad_lat"] = lat
            self.state["pad_lon"] = lon
            self.state["pad_alt"] = alt
        self.add_event("PAD_CONFIG", f"Launch pad coordinates set to ({lat:.6f}, {lon:.6f}, {alt:.1f}m)")

    def set_current_as_pad(self):
        with self.lock:
            if self.state["has_fix"] and self.state["latitude"] != 0:
                self.pad_lat = self.state["latitude"]
                self.pad_lon = self.state["longitude"]
                self.pad_alt = self.state["altitude_msl"]
                self.pad_locked = True
                self.state["pad_lat"] = self.pad_lat
                self.state["pad_lon"] = self.pad_lon
                self.state["pad_alt"] = self.pad_alt
                self.add_event("PAD_LOCKED", f"Ground station pad set to GPS fix: ({self.pad_lat:.6f}, {self.pad_lon:.6f}, {self.pad_alt:.1f}m)")
                return True
        return False

    def reset_flight_data(self):
        with self.lock:
            self.mission_start_time = None
            self.is_mission_active = False
            self.history.clear()
            self.events.clear()
            self._max_alt_msl = self.state.get("altitude_msl", 0.0)
            self._max_alt_agl = 0.0
            self._max_velocity = 0.0
            self._max_vvi = 0.0
            self.sim_time = 0.0
            self.state["max_altitude_msl"] = self._max_alt_msl
            self.state["max_altitude_agl"] = 0.0
            self.state["max_velocity"] = 0.0
            self.state["flight_phase"] = "PAD"
            self.state["met_seconds"] = 0.0
            self.state["met_formatted"] = "T+00:00:00.0"
        self.add_event("RESET", "Flight telemetry and mission metrics reset.")

    def parse_nmea_sentence(self, line: str):
        line = line.strip()
        if not line:
            return

        self.add_raw_nmea(line)

        try:
            msg = pynmea2.parse(line)
        except Exception:
            try:
                msg = pynmea2.parse(line.split('*')[0], check=False)
            except Exception:
                return

        now = time.time()
        self.last_packet_time = now
        self.packet_count += 1
        self._packet_timestamps.append(now)
        # Keep packet timestamps within last 1 second to calculate Hz
        cutoff = now - 1.0
        self._packet_timestamps = [t for t in self._packet_timestamps if t >= cutoff]
        self.packet_rate = len(self._packet_timestamps)

        with self.lock:
            current = dict(self.state)

            # Update UTC time
            if hasattr(msg, 'timestamp') and msg.timestamp:
                current["timestamp_utc"] = msg.timestamp.strftime("%H:%M:%S.%f")[:-3]
            else:
                current["timestamp_utc"] = datetime.now(timezone.utc).strftime("%H:%M:%S.%f")[:-3]

            dt = max(now - self._last_time, 0.001)

            # Parse GGA
            if isinstance(msg, pynmea2.GGA):
                if msg.latitude and msg.longitude:
                    current["latitude"] = float(msg.latitude)
                    current["longitude"] = float(msg.longitude)
                
                if msg.altitude is not None:
                    current["altitude_msl"] = float(msg.altitude)
                
                if msg.num_sats:
                    current["satellites"] = int(msg.num_sats)
                
                if msg.horizontal_dil:
                    current["hdop"] = float(msg.horizontal_dil)

                qual = int(msg.gps_qual) if msg.gps_qual is not None else 0
                current["fix_quality_code"] = qual
                current["fix_quality"] = {
                    0: "No Fix",
                    1: "GPS 3D Fix",
                    2: "DGPS Fix",
                    4: "RTK Fixed",
                    5: "RTK Float"
                }.get(qual, "Fix (Other)")
                current["has_fix"] = qual > 0

            # Parse RMC
            elif isinstance(msg, pynmea2.RMC):
                if msg.latitude and msg.longitude:
                    current["latitude"] = float(msg.latitude)
                    current["longitude"] = float(msg.longitude)
                if msg.spd_over_grnd is not None:
                    # knots to m/s
                    current["ground_speed"] = float(msg.spd_over_grnd) * 0.514444
                if msg.true_course is not None:
                    current["heading"] = float(msg.true_course)
                current["has_fix"] = (msg.status == 'A')

            # Parse VTG
            elif isinstance(msg, pynmea2.VTG):
                if msg.spd_over_grnd_kmph is not None:
                    current["ground_speed"] = float(msg.spd_over_grnd_kmph) / 3.6
                elif msg.spd_over_grnd_kts is not None:
                    current["ground_speed"] = float(msg.spd_over_grnd_kts) * 0.514444
                if msg.true_track is not None:
                    current["heading"] = float(msg.true_track)

            # Parse GSA
            elif isinstance(msg, pynmea2.GSA):
                if msg.hdop:
                    current["hdop"] = float(msg.hdop)
                if msg.vdop:
                    current["vdop"] = float(msg.vdop)

            # If Pad isn't locked yet and we have a valid 3D fix, initialize Pad
            if not self.pad_locked and current["has_fix"] and current["latitude"] != 0:
                self.pad_lat = current["latitude"]
                self.pad_lon = current["longitude"]
                self.pad_alt = current["altitude_msl"]
                self.pad_locked = True
                current["pad_lat"] = self.pad_lat
                current["pad_lon"] = self.pad_lon
                current["pad_alt"] = self.pad_alt

            # Compute relative altitude AGL
            pad_alt = self.pad_alt if self.pad_alt is not None else 0.0
            current["altitude_agl"] = max(0.0, current["altitude_msl"] - pad_alt)

            # Calculate Vertical Velocity (VVI)
            if self._last_time > 0 and dt < 2.0:
                raw_vvi = (current["altitude_msl"] - self._last_alt) / dt
                # Exponential smoothing
                current["vertical_speed"] = current["vertical_speed"] * 0.7 + raw_vvi * 0.3
            else:
                current["vertical_speed"] = 0.0

            # Compute Total 3D Velocity
            current["total_velocity"] = math.sqrt(
                current["ground_speed"] ** 2 + current["vertical_speed"] ** 2
            )

            # Track records
            if current["altitude_msl"] > self._max_alt_msl:
                self._max_alt_msl = current["altitude_msl"]
            if current["altitude_agl"] > self._max_alt_agl:
                self._max_alt_agl = current["altitude_agl"]
            if current["total_velocity"] > self._max_velocity:
                self._max_velocity = current["total_velocity"]

            current["max_altitude_msl"] = self._max_alt_msl
            current["max_altitude_agl"] = self._max_alt_agl
            current["max_velocity"] = self._max_velocity

            # Calculate Distance & Bearing from Ground Station / Launch Pad
            if self.pad_lat is not None and self.pad_lon is not None and current["latitude"] != 0:
                dist, bearing = haversine_distance_and_bearing(
                    self.pad_lat, self.pad_lon, current["latitude"], current["longitude"]
                )
                current["pad_distance"] = dist
                current["pad_bearing"] = bearing
                slant, elev = calculate_slant_range_and_elevation(dist, current["altitude_agl"])
                current["pad_slant_range"] = slant
                current["pad_elevation"] = elev

            # Mission Elapsed Time & Flight Phase state machine
            if not self.is_mission_active:
                # Detect launch: vertical speed > 10 m/s or altitude AGL > 30 m
                if current["altitude_agl"] > 30.0 or current["vertical_speed"] > 10.0:
                    self.is_mission_active = True
                    self.mission_start_time = now
                    current["flight_phase"] = "BOOST"
                    self.add_event("LAUNCH", f"Launch detected! Rocket airborne at {current['altitude_agl']:.1f}m AGL")
            
            if self.is_mission_active and self.mission_start_time:
                met = now - self.mission_start_time
                current["met_seconds"] = met
                mins, secs = divmod(met, 60)
                hrs, mins = divmod(mins, 60)
                current["met_formatted"] = f"T+{int(hrs):02d}:{int(mins):02d}:{secs:04.1f}"

                # Update flight phase
                old_phase = current["flight_phase"]
                if current["vertical_speed"] > 15.0:
                    current["flight_phase"] = "BOOST"
                elif current["vertical_speed"] > 1.0:
                    current["flight_phase"] = "COAST"
                elif -3.0 <= current["vertical_speed"] <= 1.0 and current["altitude_agl"] > 100.0:
                    current["flight_phase"] = "APOGEE"
                elif current["vertical_speed"] < -15.0:
                    current["flight_phase"] = "DROGUE"
                elif current["vertical_speed"] < -1.0:
                    current["flight_phase"] = "MAIN"
                elif current["altitude_agl"] < 15.0 and met > 10.0:
                    current["flight_phase"] = "LANDED"

                if current["flight_phase"] != old_phase:
                    self.add_event(
                        "PHASE_CHANGE", 
                        f"Flight phase changed to {current['flight_phase']} (Alt: {current['altitude_agl']:.1f}m AGL, Vspd: {current['vertical_speed']:.1f}m/s)",
                        {"old": old_phase, "new": current["flight_phase"], "alt": current["altitude_agl"]}
                    )

            # Signal Status
            if self.is_simulating:
                current["signal_status"] = "SIMULATION"
            elif self.is_connected:
                current["signal_status"] = "LIVE"
            else:
                current["signal_status"] = "DISCONNECTED"

            current["packet_rate"] = self.packet_rate
            current["connected"] = self.is_connected
            current["simulating"] = self.is_simulating

            self._last_time = now
            self._last_alt = current["altitude_msl"]
            self._last_lat = current["latitude"]
            self._last_lon = current["longitude"]

            self.state = current

            # Save to telemetry history (throttle history recording to ~2 Hz or significant delta)
            hist_item = {
                "time": current["timestamp_utc"],
                "met": current["met_seconds"],
                "alt_msl": round(current["altitude_msl"], 1),
                "alt_agl": round(current["altitude_agl"], 1),
                "vspeed": round(current["vertical_speed"], 1),
                "gspeed": round(current["ground_speed"], 1),
                "tot_vel": round(current["total_velocity"], 1),
                "lat": round(current["latitude"], 6),
                "lon": round(current["longitude"], 6),
                "sats": current["satellites"],
                "phase": current["flight_phase"]
            }
            self.history.append(hist_item)
            if len(self.history) > self.max_history:
                self.history.pop(0)

        # Broadcast update to web clients
        self.broadcast_state(self.state)

    def get_serial_ports(self) -> List[Dict[str, str]]:
        ports = []
        for port in serial.tools.list_ports.comports():
            ports.append({
                "device": port.device,
                "description": port.description,
                "hwid": port.hwid
            })
        return ports

    def connect_serial(self, port: str = "/dev/ttyUSB0", baud_rate: int = 9600) -> bool:
        self.disconnect_serial()
        self.stop_simulation()

        self.serial_port = port
        self.baud_rate = baud_rate
        self.serial_running = True

        try:
            self.serial_handle = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
            self.is_connected = True
            self.reader_thread = threading.Thread(target=self._serial_worker, daemon=True)
            self.reader_thread.start()
            self.add_event("SERIAL_CONNECT", f"Connected to GPS on {self.serial_port} at {self.baud_rate} baud")
            return True
        except Exception as e:
            self.is_connected = False
            self.serial_running = False
            self.add_event("SERIAL_ERROR", f"Failed to open {self.serial_port}: {str(e)}")
            return False

    def disconnect_serial(self):
        self.serial_running = False
        if self.serial_handle:
            try:
                self.serial_handle.close()
            except Exception:
                pass
            self.serial_handle = None
        self.is_connected = False
        self.add_event("SERIAL_DISCONNECT", "Disconnected from serial port")

    def _serial_worker(self):
        while self.serial_running and self.serial_handle and self.serial_handle.is_open:
            try:
                line = self.serial_handle.readline()
                if not line:
                    continue
                decoded = line.decode('ascii', errors='replace').strip()
                if decoded.startswith('$'):
                    self.parse_nmea_sentence(decoded)
            except Exception as e:
                time.sleep(0.1)
        self.is_connected = False

    def start_simulation(self):
        """
        Starts a realistic rocket flight simulator producing high-fidelity NMEA sentences.
        Simulates: Pre-launch pad, ignition & boost phase (0-15s), coasting to apogee (15-42s, ~4200m),
        drogue deployment & descent (42-120s), main parachute deployment (120-180s), and landing.
        """
        self.disconnect_serial()
        self.stop_simulation()

        self.is_simulating = True
        self.sim_time = 0.0
        self.reset_flight_data()
        self.mission_start_time = time.time()
        self.is_mission_active = True

        self.sim_thread = threading.Thread(target=self._sim_worker, daemon=True)
        self.sim_thread.start()
        self.add_event("SIM_START", "Rocket flight simulation started.")

    def stop_simulation(self):
        self.is_simulating = False
        if self.sim_thread and self.sim_thread.is_alive():
            self.sim_thread.join(timeout=1.0)
        self.sim_thread = None

    def _sim_worker(self):
        hz = 5.0
        dt = 1.0 / hz
        t = 0.0

        # Base coordinates (Spaceport America or configured pad)
        base_lat = self.pad_lat or 32.9904
        base_lon = self.pad_lon or -106.9750
        base_alt = self.pad_alt or 1400.0

        # Simulation parameters
        boost_duration = 14.0   # seconds
        apogee_time = 42.0      # seconds
        max_agl = 4500.0        # meters (~14,700 ft)
        drogue_descent_rate = 35.0  # m/s
        main_deploy_alt = 450.0 # meters AGL
        main_descent_rate = 6.5 # m/s
        
        # Wind drift: drift north-east
        drift_rate_lat = 0.000008  # deg/s
        drift_rate_lon = 0.000012  # deg/s

        current_alt_agl = 0.0
        current_vspeed = 0.0
        lat = base_lat
        lon = base_lon

        while self.is_simulating:
            start_loop = time.time()
            t += dt
            self.sim_time = t

            # Compute rocket trajectory dynamics
            if t < 3.0:
                # Pre-launch on pad
                current_alt_agl = 0.0
                current_vspeed = 0.0
                lat = base_lat
                lon = base_lon
            elif t < 3.0 + boost_duration:
                # Boost phase (constant high acceleration)
                tau = (t - 3.0) / boost_duration
                # Peak vertical speed at burnout ~ 280 m/s
                current_vspeed = 280.0 * math.sin(tau * math.pi / 2.0)
                current_alt_agl = 0.5 * (280.0 / boost_duration) * ((t - 3.0) ** 2)
                lat = base_lat + drift_rate_lat * (t - 3.0) * 0.4
                lon = base_lon + drift_rate_lon * (t - 3.0) * 0.4
            elif t < apogee_time:
                # Coast phase to apogee
                coast_tau = (t - (3.0 + boost_duration)) / (apogee_time - (3.0 + boost_duration))
                current_vspeed = 280.0 * (1.0 - coast_tau)
                # Reaches max_agl at apogee
                burnout_alt = 0.5 * (280.0 / boost_duration) * (boost_duration ** 2)
                current_alt_agl = burnout_alt + (max_agl - burnout_alt) * math.sin(coast_tau * math.pi / 2.0)
                lat = base_lat + drift_rate_lat * (t - 3.0)
                lon = base_lon + drift_rate_lon * (t - 3.0)
            else:
                # Descent phase
                time_since_apogee = t - apogee_time
                if current_alt_agl > main_deploy_alt:
                    # Drogue parachute
                    current_vspeed = -drogue_descent_rate
                    current_alt_agl = max(main_deploy_alt, max_agl - drogue_descent_rate * time_since_apogee)
                else:
                    # Main parachute
                    current_vspeed = -main_descent_rate
                    current_alt_agl = max(0.0, current_alt_agl - main_descent_rate * dt)

                if current_alt_agl <= 0.0:
                    current_alt_agl = 0.0
                    current_vspeed = 0.0
                
                lat = base_lat + drift_rate_lat * (t - 3.0) * 1.5
                lon = base_lon + drift_rate_lon * (t - 3.0) * 1.5

            alt_msl = base_alt + current_alt_agl
            heading = 42.0  # Drift course heading
            ground_speed = 12.5 if current_alt_agl > 0 else 0.0

            # Generate NMEA GGA sentence
            now_dt = datetime.now(timezone.utc)
            time_str = now_dt.strftime("%H%M%S.%f")[:9]
            
            # Format Lat/Lon into NMEA ddmm.mmmm format
            lat_deg = int(abs(lat))
            lat_min = (abs(lat) - lat_deg) * 60.0
            lat_dir = 'N' if lat >= 0 else 'S'
            lat_nmea = f"{lat_deg:02d}{lat_min:07.4f}"

            lon_deg = int(abs(lon))
            lon_min = (abs(lon) - lon_deg) * 60.0
            lon_dir = 'E' if lon >= 0 else 'W'
            lon_nmea = f"{lon_deg:03d}{lon_min:07.4f}"

            gga_body = f"GNGGA,{time_str},{lat_nmea},{lat_dir},{lon_nmea},{lon_dir},1,14,0.8,{alt_msl:.1f},M,-21.2,M,,"
            # Compute XOR checksum
            cs = 0
            for char in gga_body:
                cs ^= ord(char)
            gga_sentence = f"${gga_body}*{cs:02X}"

            # Generate NMEA RMC sentence
            speed_knots = ground_speed / 0.514444
            date_str = now_dt.strftime("%d%m%y")
            rmc_body = f"GNRMC,{time_str},A,{lat_nmea},{lat_dir},{lon_nmea},{lon_dir},{speed_knots:.1f},{heading:.1f},{date_str},,,A"
            cs_rmc = 0
            for char in rmc_body:
                cs_rmc ^= ord(char)
            rmc_sentence = f"${rmc_body}*{cs_rmc:02X}"

            # Feed to parser
            self.parse_nmea_sentence(gga_sentence)
            self.parse_nmea_sentence(rmc_sentence)

            elapsed = time.time() - start_loop
            sleep_time = max(0.01, dt - elapsed)
            time.sleep(sleep_time)

    def export_csv(self) -> str:
        with self.lock:
            rows = ["MET (s),Time UTC,Alt MSL (m),Alt AGL (m),V-Speed (m/s),G-Speed (m/s),Tot Velocity (m/s),Latitude,Longitude,Satellites,Phase"]
            for h in self.history:
                rows.append(f"{h['met']:.2f},{h['time']},{h['alt_msl']},{h['alt_agl']},{h['vspeed']},{h['gspeed']},{h['tot_vel']},{h['lat']:.6f},{h['lon']:.6f},{h['sats']},{h['phase']}")
            return "\n".join(rows)

    def export_json(self) -> List[Dict[str, Any]]:
        with self.lock:
            return list(self.history)


# Global singleton instance
telemetry_manager = TelemetryManager()
