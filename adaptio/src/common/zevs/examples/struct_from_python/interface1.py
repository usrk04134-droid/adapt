from dataclasses import dataclass


@dataclass
class TestS1:
    message_id: int = 0x03000100
    descriptor: str = "if?I"
    i1: int = 0
    f1: float = 0.0
    b1: bool = False
    u1: int = 0


@dataclass
class TestS2:
    message_id: int = 0x03000101
    descriptor: str = "dq"
    d1: float = 0.0
    i1: int = 0
