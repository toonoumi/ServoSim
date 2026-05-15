import asyncio
import json
import os
import threading
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .physics import ServoMotor


class SocketServer:
    DEFAULT_PATH = "/tmp/servosim.sock"

    def __init__(self, motor: "ServoMotor", path: str = DEFAULT_PATH):
        self._motor = motor
        self._path = path
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._tx_queue: asyncio.Queue | None = None
        self._writers: set = set()
        self._server = None
        self._client_count: int = 0
        self._stop_event: asyncio.Event | None = None

    @property
    def socket_path(self) -> str:
        return self._path

    @property
    def client_count(self) -> int:
        return self._client_count

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run_loop, daemon=True, name="SocketServer")
        self._thread.start()

    def stop(self) -> None:
        if self._loop and self._loop.is_running() and self._stop_event is not None:
            self._loop.call_soon_threadsafe(self._stop_event.set)
        if self._thread:
            self._thread.join(timeout=2.0)
        self._cleanup_socket()

    def push_telemetry(self, payload: dict) -> None:
        if self._loop and self._loop.is_running() and self._tx_queue is not None:
            line = json.dumps(payload) + "\n"
            self._loop.call_soon_threadsafe(self._tx_queue.put_nowait, line)

    def _cleanup_socket(self) -> None:
        try:
            os.unlink(self._path)
        except OSError:
            pass

    def _run_loop(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._tx_queue = asyncio.Queue()
        self._stop_event = asyncio.Event()
        self._cleanup_socket()
        try:
            self._loop.run_until_complete(self._serve())
        finally:
            self._cleanup_socket()
            self._loop.close()

    async def _serve(self) -> None:
        self._server = await asyncio.start_unix_server(
            self._handle_client, path=self._path
        )
        broadcast_task = self._loop.create_task(self._broadcast_loop())
        async with self._server:
            await self._stop_event.wait()
            self._server.close()
        broadcast_task.cancel()
        try:
            await broadcast_task
        except asyncio.CancelledError:
            pass

    async def _broadcast_loop(self) -> None:
        while True:
            line = await self._tx_queue.get()
            dead = set()
            for writer in list(self._writers):
                try:
                    writer.write(line.encode())
                    await writer.drain()
                except (ConnectionResetError, BrokenPipeError):
                    dead.add(writer)
            for w in dead:
                self._writers.discard(w)
                self._client_count = len(self._writers)

    async def _handle_client(self, reader: asyncio.StreamReader,
                               writer: asyncio.StreamWriter) -> None:
        self._writers.add(writer)
        self._client_count = len(self._writers)
        try:
            while True:
                line = await reader.readline()
                if not line:
                    break
                try:
                    msg = json.loads(line)
                except json.JSONDecodeError:
                    continue
                self._apply_command(msg)
        except (ConnectionResetError, asyncio.IncompleteReadError):
            pass
        finally:
            self._writers.discard(writer)
            self._client_count = len(self._writers)
            try:
                writer.close()
                await writer.wait_closed()
            except Exception:
                pass

    def _apply_command(self, msg: dict) -> None:
        motor = self._motor
        if "kp" in msg or "ki" in msg or "kd" in msg:
            kp = float(msg.get("kp", motor.pid.kp))
            ki = float(msg.get("ki", motor.pid.ki))
            kd = float(msg.get("kd", motor.pid.kd))
            motor.pid.set_gains(kp, ki, kd)
        if "target" in msg:
            motor.set_target(float(msg["target"]))
        if "payload" in msg:
            motor.set_payload(float(msg["payload"]))
        if "direction" in msg:
            tilt = float(msg["tilt_angle"]) if "tilt_angle" in msg else None
            try:
                motor.set_payload_direction(msg["direction"], tilt)
            except ValueError:
                pass  # ignore unknown direction strings
