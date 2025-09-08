from typing import Callable, Any
import zmq
import json
import time


class Receiver:
    def __init__(self, socket: zmq.Socket, timeout_ms: int):
        self.socket = socket
        self.timeout_ms = timeout_ms

    def poll(self) -> Any | None:
        if self.socket.poll(self.timeout_ms, zmq.POLLIN):
            [topic, packet] = self.socket.recv_multipart(zmq.NOBLOCK)
            message = json.loads(packet)
            return message
        return None

    def poll_specific(self, expected_name: str) -> Any | None:
        message = self.poll()
        if message is not None and message["name"] == expected_name:
            return message
        return None

    def verify(self, message_name: str, func: Callable[[Any], bool]) -> bool:
        message = self.poll()
        if message is None:
            return False
        if message['name'] != message_name:
            print("message.py - Expected message:" +  message_name + ", got:" + message['name'])
        if message['name'] == message_name:
            payload = message['payload']
            return func(payload)
        return False


class Sender:
    def __init__(self, socket: zmq.Socket):
        self.socket = socket

    def send(self, message):
        packet = json.dumps(message).encode()
        self.socket.send_multipart([b"adaptio_io", packet])


class LoopUntil:
    def __init__(self, max_duration):
        self.max_duration = max_duration

    def __iter__(self):
        self.started_at = time.time()
        return self

    def __next__(self):
        # Poll period is 250ms, not too short and not too long
        time.sleep(0.25)
        current_time = time.time()
        duration = current_time - self.started_at
        if duration > self.max_duration:
            raise StopIteration
        else:
            return duration
