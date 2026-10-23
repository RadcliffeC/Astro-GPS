import time
import pytest
from fastapi.testclient import TestClient
from data_parser.telemetry_manager import (
    TelemetryManager,
    haversine_distance_and_bearing,
    calculate_slant_range_and_elevation
)
from frontend.api import app


def test_distance_and_bearing_calculation():
    # Test distance between Spaceport America and a point ~1km north
    lat1, lon1 = 32.9904, -106.9750
    lat2, lon2 = 32.9994, -106.9750
    dist, bearing = haversine_distance_and_bearing(lat1, lon1, lat2, lon2)
    assert 900 < dist < 1100
    assert abs(bearing - 0.0) < 1.0 or abs(bearing - 360.0) < 1.0


def test_slant_range_and_elevation():
    slant, elev = calculate_slant_range_and_elevation(1000.0, 1000.0)
    assert round(slant, 1) == 1414.2
    assert round(elev, 1) == 45.0


def test_nmea_parsing_and_telemetry():
    tm = TelemetryManager()
    
    # GGA sentence
    gga = "$GNGGA,182432.00,3259.4240,N,10658.5000,W,1,12,0.8,1450.5,M,-21.2,M,,*7C"
    tm.parse_nmea_sentence(gga)

    assert tm.state["has_fix"] is True
    assert tm.state["satellites"] == 12
    assert tm.state["hdop"] == 0.8
    assert round(tm.state["latitude"], 3) == 32.990
    assert round(tm.state["longitude"], 3) == -106.975
    assert tm.state["altitude_msl"] == 1450.5

    # RMC sentence
    rmc = "$GNRMC,182432.00,A,3259.4240,N,10658.5000,W,25.0,045.0,231026,,,A*45"
    tm.parse_nmea_sentence(rmc)

    assert tm.state["heading"] == 45.0
    assert round(tm.state["ground_speed"], 1) == round(25.0 * 0.514444, 1)

    # VTG sentence
    vtg = "$GNVTG,045.0,T,,M,25.0,N,46.3,K,A*23"
    tm.parse_nmea_sentence(vtg)
    assert tm.state["heading"] == 45.0

    # GSA sentence
    gsa = "$GNGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.2,0.8,0.9*1A"
    tm.parse_nmea_sentence(gsa)
    assert tm.state["hdop"] == 0.8
    assert tm.state["vdop"] == 0.9


def test_flight_events_and_apogee():
    tm = TelemetryManager()
    tm.set_pad_coordinates(32.9904, -106.9750, 1400.0)
    
    # Feed ascent sequence
    tm.parse_nmea_sentence("$GNGGA,120000.00,3259.4240,N,10658.5000,W,1,10,0.9,1500.0,M,0,M,,*4D")
    tm.parse_nmea_sentence("$GNGGA,120005.00,3259.4240,N,10658.5000,W,1,10,0.9,2500.0,M,0,M,,*4C")
    tm.parse_nmea_sentence("$GNGGA,120010.00,3259.4240,N,10658.5000,W,1,10,0.9,4500.0,M,0,M,,*4A")

    assert tm.state["max_altitude_msl"] >= 4500.0
    assert tm.state["max_altitude_agl"] >= 3100.0
    assert len(tm.events) > 0


def test_simulation_mode():
    tm = TelemetryManager()
    tm.start_simulation()
    assert tm.is_simulating is True
    time.sleep(0.5)
    assert tm.state["signal_status"] == "SIMULATION"
    assert tm.state["packet_rate"] > 0
    tm.stop_simulation()
    assert tm.is_simulating is False


def test_lock_pad_from_current_fix():
    tm = TelemetryManager()
    # Before fix
    assert tm.set_current_as_pad() is False
    
    # After fix
    tm.parse_nmea_sentence("$GNGGA,182432.00,3259.4240,N,10658.5000,W,1,12,0.8,1450.5,M,-21.2,M,,*7C")
    assert tm.set_current_as_pad() is True
    assert round(tm.pad_lat, 3) == 32.990
    assert round(tm.pad_lon, 3) == -106.975


def test_fastapi_rest_endpoints():
    with TestClient(app) as client:
        # Ping
        r = client.get("/ping")
        assert r.status_code == 200
        assert r.json()["ping"] == "pong"

        # Root static / fallback
        r = client.get("/")
        assert r.status_code == 200

        # Telemetry
        r = client.get("/api/telemetry")
        assert r.status_code == 200
        data = r.json()
        assert "altitude_msl" in data
        assert "flight_phase" in data

        # History
        r = client.get("/api/history")
        assert r.status_code == 200
        assert "history" in r.json()

        # Events
        r = client.get("/api/events")
        assert r.status_code == 200
        assert isinstance(r.json(), list)

        # Raw NMEA
        r = client.get("/api/raw_nmea")
        assert r.status_code == 200
        assert isinstance(r.json(), list)

        # Ports
        r = client.get("/api/ports")
        assert r.status_code == 200
        assert isinstance(r.json(), list)

        # Set Pad
        r = client.post("/api/pad/set", json={"latitude": 33.123, "longitude": -106.456, "altitude": 1200.0})
        assert r.status_code == 200
        assert r.json()["status"] == "pad_configured"

        # Simulation start & stop endpoints
        r = client.post("/api/simulation/start")
        assert r.status_code == 200
        time.sleep(0.2)
        r = client.post("/api/simulation/stop")
        assert r.status_code == 200

        # Reset
        r = client.post("/api/reset")
        assert r.status_code == 200
        assert r.json()["status"] == "reset_successful"

        # Export CSV
        r = client.get("/api/export/csv")
        assert r.status_code == 200
        assert "MET" in r.text

        # Export JSON
        r = client.get("/api/export/json")
        assert r.status_code == 200
        assert isinstance(r.json(), list)


def test_websocket_telemetry_stream():
    with TestClient(app) as client:
        with client.websocket_connect("/ws/telemetry") as websocket:
            data = websocket.receive_json()
            assert data["type"] == "INITIAL_SYNC"
            assert "state" in data
            assert "history" in data
