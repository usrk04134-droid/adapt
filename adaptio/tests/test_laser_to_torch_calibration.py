import time
from message import Receiver, Sender, LoopUntil
from message_factory import (LaserTorchCalSet, LaserTorchCalGet, GetAdaptioVersion)

def test_calibration(zmq_sockets, adaptio_app, config_file):
    time.sleep(1)

    request_socket, reply_socket = zmq_sockets

    receive_timeout_ms = 200
    receiver = Receiver(reply_socket, receive_timeout_ms)
    sender = Sender(request_socket)

    adaptio_app.run(["--config-file", "tests/configs/sil/configuration_laser_to_torch.yaml"])

    # Test unstable without a sleep here
    time.sleep(10)

    # Check adaptio started by print adaptio version
    for _ in LoopUntil(15.0):
        sender.send(GetAdaptioVersion())
        version_rsp = receiver.poll_specific("GetAdaptioVersionRsp")
        if version_rsp is not None:
            print("Adaptio version: " + version_rsp["payload"]["version"])
            break
        time.sleep(1)
    else:
        assert False, "GetAdaptioVersion not received"

     # Laser to Torch Calibration v2: Set and Get
    ltc_set_msg = LaserTorchCalSet()
    ltc_set_msg["payload"]["distanceLaserTorch"] = 150.0
    ltc_set_msg["payload"]["stickout"] = 25.0
    ltc_set_msg["payload"]["scannerMountAngle"] = 0.26

    sender.send(ltc_set_msg)

    # Wait for LaserTorchCalSetRsp
    for _ in LoopUntil(10.0):
        if receiver.verify("LaserTorchCalSetRsp",
                           lambda payload: payload.get("result") == "ok"):
            break
    else:
        assert False, "LaserTorchCalSetRsp not received"

    # Request LaserTorchCalGet and verify values present
    sender.send(LaserTorchCalGet())
    for _ in LoopUntil(10.0):
        if receiver.verify("LaserTorchCalGetRsp",
                           lambda payload: payload.get("result") == "ok" and
                                           "distanceLaserTorch" in payload and
                                           "stickout" in payload and
                                           "scannerMountAngle" in payload):
            break
    else:
        assert False, "LaserTorchCalGetRsp not received"

    time.sleep(1)
    assert True, "LaserTorchCal v2 test completed"

    adaptio_app.quit(None)

    time.sleep(5)
