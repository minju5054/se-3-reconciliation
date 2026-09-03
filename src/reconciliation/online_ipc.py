"""Versioned standard-library IPC for isolated Isaac and LightNav processes."""

from __future__ import annotations

import json
import socket
import struct
import threading
from typing import Any, Mapping

import numpy as np
from numpy.typing import ArrayLike


PROTOCOL_NAME = "se3-reconciliation-lightnav-ipc"
PROTOCOL_VERSION = 1
RGB_FORMAT = "HWC uint8 RGB"
MAX_HEADER_BYTES = 1_048_576
MAX_PAYLOAD_BYTES = 128 * 1024 * 1024
_LENGTHS = struct.Struct("!IQ")


def validate_rgb_frame(frame: ArrayLike) -> np.ndarray:
    """Return a contiguous RGB view after enforcing the wire contract."""

    value = np.asarray(frame)
    if value.ndim != 3 or value.shape[0] < 1 or value.shape[1] < 1 or value.shape[2] != 3:
        raise ValueError("RGB frame must have non-empty shape (H, W, 3)")
    if value.dtype != np.uint8:
        raise ValueError("RGB frame dtype must be uint8")
    return np.ascontiguousarray(value)


def _strict_object(encoded: bytes) -> dict[str, Any]:
    try:
        value = json.loads(
            encoded.decode("utf-8"),
            parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)),
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("IPC header is not strict UTF-8 JSON") from error
    if not isinstance(value, dict):
        raise ValueError("IPC header must contain a JSON object")
    if value.get("protocol") != PROTOCOL_NAME or value.get("version") != PROTOCOL_VERSION:
        raise ValueError("IPC protocol name/version mismatch")
    message_type = value.get("type")
    if not isinstance(message_type, str) or not message_type:
        raise ValueError("IPC message type must be a non-empty string")
    return value


def encode_message(header: Mapping[str, Any], payload: bytes = b"") -> bytes:
    """Encode one explicit length-prefixed message for socket transport."""

    if not isinstance(payload, bytes):
        raise TypeError("IPC payload must be bytes")
    document = dict(header)
    document["protocol"] = PROTOCOL_NAME
    document["version"] = PROTOCOL_VERSION
    document["payload_nbytes"] = len(payload)
    encoded = json.dumps(
        document,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    if len(encoded) > MAX_HEADER_BYTES:
        raise ValueError("IPC header exceeds size limit")
    if len(payload) > MAX_PAYLOAD_BYTES:
        raise ValueError("IPC payload exceeds size limit")
    return _LENGTHS.pack(len(encoded), len(payload)) + encoded + payload


def decode_message_bytes(encoded: bytes) -> tuple[dict[str, Any], bytes]:
    """Decode one complete message; useful for fixtures and protocol tests."""

    if len(encoded) < _LENGTHS.size:
        raise ValueError("IPC message is truncated")
    header_size, payload_size = _LENGTHS.unpack(encoded[: _LENGTHS.size])
    if header_size > MAX_HEADER_BYTES or payload_size > MAX_PAYLOAD_BYTES:
        raise ValueError("IPC message exceeds size limit")
    expected = _LENGTHS.size + header_size + payload_size
    if len(encoded) != expected:
        raise ValueError("IPC message length does not match framing")
    header_start = _LENGTHS.size
    header = _strict_object(encoded[header_start : header_start + header_size])
    if int(header.get("payload_nbytes", -1)) != payload_size:
        raise ValueError("IPC header payload length mismatch")
    return header, encoded[header_start + header_size :]


def _recv_exact(connection: socket.socket, count: int) -> bytes:
    parts: list[bytes] = []
    remaining = count
    while remaining:
        part = connection.recv(remaining)
        if not part:
            raise EOFError("IPC peer closed while a message was in flight")
        parts.append(part)
        remaining -= len(part)
    return b"".join(parts)


def send_message(
    connection: socket.socket,
    header: Mapping[str, Any],
    payload: bytes = b"",
) -> None:
    connection.sendall(encode_message(header, payload))


def receive_message(connection: socket.socket) -> tuple[dict[str, Any], bytes]:
    prefix = _recv_exact(connection, _LENGTHS.size)
    header_size, payload_size = _LENGTHS.unpack(prefix)
    if header_size > MAX_HEADER_BYTES or payload_size > MAX_PAYLOAD_BYTES:
        raise ValueError("IPC message exceeds size limit")
    header = _strict_object(_recv_exact(connection, header_size))
    if int(header.get("payload_nbytes", -1)) != payload_size:
        raise ValueError("IPC header payload length mismatch")
    return header, _recv_exact(connection, payload_size)


def rgb_message(
    frame: ArrayLike,
    *,
    request_id: int,
    frame_index: int,
    sim_time_s: float,
) -> tuple[dict[str, Any], bytes]:
    rgb = validate_rgb_frame(frame)
    header = {
        "type": "observe_rgb",
        "request_id": int(request_id),
        "frame_index": int(frame_index),
        "sim_time_s": float(sim_time_s),
        "shape": list(rgb.shape),
        "dtype": str(rgb.dtype),
        "rgb_format": RGB_FORMAT,
    }
    return header, rgb.tobytes(order="C")


def decode_rgb_message(header: Mapping[str, Any], payload: bytes) -> np.ndarray:
    if header.get("type") != "observe_rgb":
        raise ValueError("message is not an RGB observation")
    if header.get("rgb_format") != RGB_FORMAT or header.get("dtype") != "uint8":
        raise ValueError("RGB format/dtype metadata is invalid")
    shape = header.get("shape")
    if (
        not isinstance(shape, list)
        or len(shape) != 3
        or any(not isinstance(item, int) for item in shape)
        or shape[2] != 3
        or shape[0] < 1
        or shape[1] < 1
    ):
        raise ValueError("RGB shape metadata is invalid")
    expected = int(np.prod(shape, dtype=np.int64))
    if len(payload) != expected:
        raise ValueError("RGB byte length differs from shape metadata")
    return validate_rgb_frame(np.frombuffer(payload, dtype=np.uint8).reshape(shape)).copy()


def array_payload(array: ArrayLike) -> tuple[dict[str, Any], bytes]:
    value = np.asarray(array)
    if value.ndim < 1 or not np.issubdtype(value.dtype, np.number):
        raise ValueError("array payload must be a non-empty numeric array")
    contiguous = np.ascontiguousarray(value)
    return {
        "array_shape": list(contiguous.shape),
        "array_dtype": str(contiguous.dtype),
    }, contiguous.tobytes(order="C")


def decode_array_payload(header: Mapping[str, Any], payload: bytes) -> np.ndarray:
    shape = header.get("array_shape")
    dtype_name = header.get("array_dtype")
    if not isinstance(shape, list) or not shape or any(
        not isinstance(item, int) or item < 1 for item in shape
    ):
        raise ValueError("array shape metadata is invalid")
    try:
        dtype = np.dtype(dtype_name)
    except TypeError as error:
        raise ValueError("array dtype metadata is invalid") from error
    expected = int(np.prod(shape, dtype=np.int64)) * dtype.itemsize
    if len(payload) != expected:
        raise ValueError("array byte length differs from metadata")
    return np.frombuffer(payload, dtype=dtype).reshape(shape).copy()


class OnlineLightNavClient:
    """Single-flight request client; prediction may run from an Isaac-side worker thread."""

    def __init__(self, socket_path: str, *, timeout_s: float = 120.0) -> None:
        self._connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._connection.settimeout(float(timeout_s))
        self._connection.connect(socket_path)
        self._next_request_id = 0
        self._lock = threading.Lock()

    def close(self) -> None:
        self._connection.close()

    def _request(
        self,
        message_type: str,
        fields: Mapping[str, Any] | None = None,
        payload: bytes = b"",
    ) -> tuple[dict[str, Any], bytes]:
        with self._lock:
            request_id = self._next_request_id
            self._next_request_id += 1
            header = {"type": message_type, "request_id": request_id, **dict(fields or {})}
            send_message(self._connection, header, payload)
            response, response_payload = receive_message(self._connection)
            if response.get("request_id") != request_id:
                raise RuntimeError("IPC response request_id mismatch")
            if response.get("type") == "error":
                raise RuntimeError(f"LightNav server error: {response.get('error')}")
            if response.get("type") != f"{message_type}_response":
                raise RuntimeError("IPC response type mismatch")
            return response, response_payload

    def server_info(self) -> dict[str, Any]:
        header, payload = self._request("server_info")
        if payload:
            raise RuntimeError("server_info response unexpectedly has a payload")
        return dict(header["server_info"])

    def reset_episode(self, instruction: str, episode_index: int) -> dict[str, Any]:
        header, payload = self._request(
            "episode_reset",
            {"instruction": instruction, "episode_index": int(episode_index)},
        )
        if payload:
            raise RuntimeError("episode_reset response unexpectedly has a payload")
        return header

    def observe(self, frame: ArrayLike, *, frame_index: int, sim_time_s: float) -> dict[str, Any]:
        request_id = self._next_request_id
        rgb_header, payload = rgb_message(
            frame,
            request_id=request_id,
            frame_index=frame_index,
            sim_time_s=sim_time_s,
        )
        # _request owns the authoritative request id; remove the provisional one.
        rgb_header.pop("request_id")
        header, response_payload = self._request("observe_rgb", rgb_header, payload)
        if response_payload:
            raise RuntimeError("observe response unexpectedly has a payload")
        return header

    def predict(self, *, prediction_kind: str) -> tuple[np.ndarray, dict[str, Any]]:
        if prediction_kind not in ("old", "new"):
            raise ValueError("prediction_kind must be 'old' or 'new'")
        header, payload = self._request("predict", {"prediction_kind": prediction_kind})
        return decode_array_payload(header, payload), header

    def shutdown_server(self) -> None:
        self._request("shutdown")
