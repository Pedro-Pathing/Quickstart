"""HTTP client for the robot's agent bridge (TeamCode/.../agent/AgentBridge.java)."""

from __future__ import annotations

import json
import os
from typing import Any

import httpx
from mcp.server.mcpserver.exceptions import ToolError


class RobotError(ToolError):
    """The bridge refused a request, or the robot could not be reached.

    A ToolError, so MCP clients see the message (mcp 2.x hides other exceptions' text).
    """


class RobotClient:
    """Thin async wrapper over the /agent/ endpoints on the Robot Controller (port 8080)."""

    def __init__(self, base_url: str | None = None, transport: httpx.AsyncBaseTransport | None = None):
        host = os.environ.get("ROBOT_HOST", "192.168.43.1")
        port = os.environ.get("ROBOT_PORT", "8080")
        self.base_url = base_url or f"http://{host}:{port}"
        self._http = httpx.AsyncClient(base_url=self.base_url, timeout=15.0, transport=transport)

    async def close(self) -> None:
        await self._http.aclose()

    async def _request(
        self, method: str, path: str, params: dict[str, Any] | None = None, body: Any = None
    ) -> httpx.Response:
        try:
            r = await self._http.request(
                method,
                path,
                params={k: v for k, v in (params or {}).items() if v is not None},
                content=None if body is None else json.dumps(body),
                headers={"Content-Type": "application/json"} if body is not None else None,
            )
        except httpx.HTTPError as e:
            raise RobotError(
                f"cannot reach the robot at {self.base_url} ({e.__class__.__name__}: {e}). "
                "Is the laptop on the robot's Wi-Fi, and is the robot app running?"
            ) from e
        if r.status_code >= 400:
            try:
                message = r.json().get("error", r.text)
            except ValueError:
                message = r.text
            if r.status_code == 404:
                message = (
                    f"{path} not found: the robot is running an APK without the agent bridge. "
                    "Install the current TeamCode build."
                )
            raise RobotError(f"{method} {path} -> {r.status_code}: {message}")
        return r

    async def get(self, path: str, **params: Any) -> Any:
        return (await self._request("GET", path, params)).json()

    async def post(self, path: str, body: Any = None, **params: Any) -> Any:
        return (await self._request("POST", path, params, body)).json()

    async def text(self, path: str, **params: Any) -> str:
        return (await self._request("GET", path, params)).text

    # --- endpoints ---------------------------------------------------------------------------

    async def status(self) -> dict:
        return await self.get("/agent/status")

    async def opmodes(self) -> list[dict]:
        return await self.get("/agent/opmodes")

    async def init_opmode(self, name: str) -> dict:
        return await self.post("/agent/opmode/init", name=name)

    async def start_opmode(self) -> dict:
        return await self.post("/agent/opmode/start")

    async def stop_opmode(self) -> dict:
        return await self.post("/agent/opmode/stop")

    async def play_gamepad(self, script: dict) -> dict:
        return await self.post("/agent/gamepad", body=script)

    async def stop_gamepad(self) -> dict:
        return await self.post("/agent/gamepad/stop")

    async def telemetry(self, since: int | None = None, max_entries: int | None = None) -> dict:
        return await self.get("/agent/telemetry", since=since, max=max_entries)

    async def hardware(self) -> dict:
        return await self.get("/agent/hardware")

    async def start_recording(self, rate_hz: float | None = None) -> dict:
        return await self.post("/agent/record/start", rate_hz=rate_hz)

    async def stop_recording(self) -> dict:
        return await self.post("/agent/record/stop")

    async def recording_csv(self) -> str:
        return await self.text("/agent/record/csv")

    async def get_config(self, key: str) -> dict:
        return await self.get("/agent/config", key=key)

    async def set_config(self, key: str, value: Any) -> dict:
        return await self.post("/agent/config", key=key, value=json.dumps(value))

    async def configurables(self, filter_text: str | None = None) -> list[dict]:
        return await self.get("/agent/configurables", filter=filter_text)

    async def get_configurable(self, key: str) -> dict:
        return await self.get("/agent/configurable", key=key)

    async def set_configurable(self, key: str, value: Any) -> dict:
        return await self.post("/agent/configurable", key=key, value=json.dumps(value))

    async def start_flight(self, label: str) -> dict:
        return await self.post("/agent/flight/start", label=label)

    async def stop_flight(self) -> dict:
        return await self.post("/agent/flight/stop")

    async def flights(self) -> list[dict]:
        return await self.get("/agent/flights")

    async def flight_chunk(self, flight_id: str, name: str, offset: int) -> tuple[bytes, int, int]:
        """Bytes of one flight file from offset: (data, next_offset, file_size)."""
        r = await self._request("GET", "/agent/flight/file", {"id": flight_id, "name": name, "offset": offset})
        data = r.content
        return data, int(r.headers.get("X-Next-Offset", offset + len(data))), int(r.headers.get("X-File-Size", 0))
