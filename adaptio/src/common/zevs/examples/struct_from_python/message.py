import struct
import copy

from dataclasses import astuple


class MessageRegistry:
    def __init__(self):
        self.msg_dict = dict()

    def add(self, message):
        self.msg_dict[message.message_id] = message

    def get(self, message_id):
        prototype = self.msg_dict.get(message_id)
        return copy.deepcopy(prototype)


def pack(obj):
    tup = astuple(obj)
    # payload is after message_id and descriptor
    payload_tup = tup[2:]
    buf = struct.pack(obj.descriptor, *payload_tup)
    return buf


def packet_from_message(obj):
    buf = pack(obj)
    id_buf = struct.pack("I", obj.message_id)
    return id_buf + buf


def unpack(obj, buf):
    payload_tup = struct.unpack(obj.descriptor, buf)
    tup = (obj.message_id, obj.descriptor) + payload_tup
    obj.__init__(*tup)


def message_from_packet(registry, packet):
    message_id_bytes = packet[:4]
    message_bytes = packet[4:]

    message_id = struct.unpack("I", message_id_bytes)[0]

    msg_struct = registry.get(message_id)
    if msg_struct is None:
        return None

    unpack(msg_struct, message_bytes)
    return msg_struct


def send_message(socket, message):
    packet = packet_from_message(message)
    socket.send(packet)


def recv_message(registry, socket):
    packet = socket.recv()
    return message_from_packet(registry, packet)
