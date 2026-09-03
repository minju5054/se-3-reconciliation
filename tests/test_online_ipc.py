import socket
import threading

import numpy as np
import pytest

from reconciliation.online_ipc import (
    PROTOCOL_NAME,
    PROTOCOL_VERSION,
    OnlineLightNavClient,
    array_payload,
    decode_array_payload,
    decode_message_bytes,
    decode_rgb_message,
    encode_message,
    receive_message,
    rgb_message,
    send_message,
    validate_rgb_frame,
)


def test_message_round_trip_and_version() -> None:
    encoded = encode_message({"type": "ping", "request_id": 3}, b"abc")
    header, payload = decode_message_bytes(encoded)
    assert header["protocol"] == PROTOCOL_NAME
    assert header["version"] == PROTOCOL_VERSION
    assert header["request_id"] == 3
    assert payload == b"abc"


def test_socket_transport_handles_framing() -> None:
    first, second = socket.socketpair()
    try:
        send_message(first, {"type": "status"}, b"payload")
        header, payload = receive_message(second)
    finally:
        first.close()
        second.close()
    assert header["type"] == "status"
    assert payload == b"payload"


def test_rgb_contract_and_round_trip() -> None:
    frame = np.arange(4 * 5 * 3, dtype=np.uint8).reshape(4, 5, 3)
    header, payload = rgb_message(frame, request_id=1, frame_index=7, sim_time_s=2.5)
    decoded = decode_rgb_message(header, payload)
    assert np.array_equal(decoded, frame)
    assert decoded.flags.c_contiguous


@pytest.mark.parametrize(
    "bad",
    (
        np.zeros((4, 5), dtype=np.uint8),
        np.zeros((4, 5, 4), dtype=np.uint8),
        np.zeros((4, 5, 3), dtype=np.float32),
    ),
)
def test_rgb_contract_rejects_shape_and_dtype(bad: np.ndarray) -> None:
    with pytest.raises(ValueError):
        validate_rgb_frame(bad)


def test_rgb_decoder_rejects_payload_length() -> None:
    header, payload = rgb_message(
        np.zeros((2, 3, 3), dtype=np.uint8), request_id=1, frame_index=0, sim_time_s=0.0
    )
    with pytest.raises(ValueError, match="byte length"):
        decode_rgb_message(header, payload[:-1])


def test_numeric_array_payload_round_trip() -> None:
    source = np.arange(30, dtype=np.float32).reshape(10, 3)
    header, payload = array_payload(source)
    decoded = decode_array_payload(header, payload)
    assert decoded.dtype == np.float32
    assert np.array_equal(decoded, source)


def test_client_request_ids_and_array_response(tmp_path) -> None:
    socket_path = str(tmp_path / "ipc.sock")
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(socket_path)
    server.listen(1)

    def serve() -> None:
        connection, _ = server.accept()
        try:
            header, _ = receive_message(connection)
            array_header, payload = array_payload(np.ones((2, 3), dtype=np.float32))
            send_message(
                connection,
                {
                    "type": "predict_response",
                    "request_id": header["request_id"],
                    **array_header,
                },
                payload,
            )
        finally:
            connection.close()
            server.close()

    thread = threading.Thread(target=serve)
    thread.start()
    client = OnlineLightNavClient(socket_path)
    try:
        actions, header = client.predict(prediction_kind="old")
    finally:
        client.close()
        thread.join()
    assert header["request_id"] == 0
    assert np.array_equal(actions, np.ones((2, 3), dtype=np.float32))
