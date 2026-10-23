import os
import sys
import asyncio
from pathlib import Path
from typing import Optional
from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, PlainTextResponse, JSONResponse
from pydantic import BaseModel

# Ensure parent directory is in sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from data_parser.telemetry_manager import telemetry_manager

STATIC_DIR = Path(__file__).resolve().parent / "static"


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Set the running event loop in telemetry_manager
    loop = asyncio.get_running_loop()
    telemetry_manager.set_event_loop(loop)
    yield
    # Shutdown cleanup
    telemetry_manager.disconnect_serial()
    telemetry_manager.stop_simulation()


app = FastAPI(title="MAX-M10S Rocket Ground Station", version="1.0.0", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class ConnectRequest(BaseModel):
    port: str = "/dev/ttyUSB0"
    baud_rate: int = 9600


class PadConfigRequest(BaseModel):
    latitude: float
    longitude: float
    altitude: float = 0.0


@app.get("/")
async def root():
    index_file = STATIC_DIR / "index.html"
    if index_file.exists():
        return FileResponse(index_file)
    return {"message": "Rocket Telemetry Ground Station API is running"}


@app.get("/ping")
async def ping():
    return {"ping": "pong", "service": "MAX-M10S Rocket Ground Station"}


@app.get("/api/telemetry")
async def get_telemetry():
    with telemetry_manager.lock:
        return telemetry_manager.state


@app.get("/api/history")
async def get_history():
    with telemetry_manager.lock:
        return {
            "count": len(telemetry_manager.history),
            "history": telemetry_manager.history
        }


@app.get("/api/events")
async def get_events():
    with telemetry_manager.lock:
        return telemetry_manager.events


@app.get("/api/raw_nmea")
async def get_raw_nmea():
    with telemetry_manager.lock:
        return telemetry_manager.raw_nmea_logs


@app.get("/api/ports")
async def list_ports():
    return telemetry_manager.get_serial_ports()


@app.post("/api/connect")
async def connect_serial(req: ConnectRequest):
    success = telemetry_manager.connect_serial(port=req.port, baud_rate=req.baud_rate)
    if not success:
        raise HTTPException(status_code=400, detail=f"Failed to connect to {req.port}")
    return {"status": "connected", "port": req.port, "baud_rate": req.baud_rate}


@app.post("/api/disconnect")
async def disconnect_serial():
    telemetry_manager.disconnect_serial()
    return {"status": "disconnected"}


@app.post("/api/simulation/start")
async def start_sim():
    telemetry_manager.start_simulation()
    return {"status": "simulation_started"}


@app.post("/api/simulation/stop")
async def stop_sim():
    telemetry_manager.stop_simulation()
    return {"status": "simulation_stopped"}


@app.post("/api/pad/set")
async def set_pad(req: PadConfigRequest):
    telemetry_manager.set_pad_coordinates(req.latitude, req.longitude, req.altitude)
    return {"status": "pad_configured", "pad": {"lat": req.latitude, "lon": req.longitude, "alt": req.altitude}}


@app.post("/api/pad/lock_current")
async def lock_current_pad():
    success = telemetry_manager.set_current_as_pad()
    if not success:
        raise HTTPException(status_code=400, detail="Cannot lock pad: No valid GPS 3D fix available")
    return {
        "status": "pad_locked",
        "pad": {
            "lat": telemetry_manager.pad_lat,
            "lon": telemetry_manager.pad_lon,
            "alt": telemetry_manager.pad_alt
        }
    }


@app.post("/api/reset")
async def reset_mission():
    telemetry_manager.reset_flight_data()
    return {"status": "reset_successful"}


@app.get("/api/export/csv")
async def export_csv():
    csv_data = telemetry_manager.export_csv()
    return PlainTextResponse(
        csv_data,
        media_type="text/csv",
        headers={"Content-Disposition": "attachment; filename=rocket_flight_telemetry.csv"}
    )


@app.get("/api/export/json")
async def export_json():
    json_data = telemetry_manager.export_json()
    return JSONResponse(
        content=json_data,
        headers={"Content-Disposition": "attachment; filename=rocket_flight_telemetry.json"}
    )


@app.websocket("/ws/telemetry")
async def websocket_telemetry(websocket: WebSocket):
    await websocket.accept()
    queue = asyncio.Queue(maxsize=100)
    telemetry_manager.add_subscriber(queue)

    try:
        # Send initial full state immediately
        with telemetry_manager.lock:
            initial_state = dict(telemetry_manager.state)
            initial_history = list(telemetry_manager.history[-200:])
            initial_events = list(telemetry_manager.events[-20:])
            initial_nmea = list(telemetry_manager.raw_nmea_logs[-20:])

        await websocket.send_json({
            "type": "INITIAL_SYNC",
            "state": initial_state,
            "history": initial_history,
            "events": initial_events,
            "raw_nmea": initial_nmea
        })

        # Keep streaming new state frames
        while True:
            # Wait for next state broadcast or handle incoming commands
            try:
                state_update = await asyncio.wait_for(queue.get(), timeout=1.0)
                await websocket.send_json({
                    "type": "TELEMETRY_UPDATE",
                    "state": state_update
                })
            except asyncio.TimeoutError:
                # Send periodic heartbeat/keepalive state
                with telemetry_manager.lock:
                    current_state = dict(telemetry_manager.state)
                await websocket.send_json({
                    "type": "HEARTBEAT",
                    "state": current_state
                })
    except (WebSocketDisconnect, asyncio.CancelledError):
        pass
    finally:
        telemetry_manager.remove_subscriber(queue)


# Mount static assets if directory exists
if STATIC_DIR.exists():
    app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")

