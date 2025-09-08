import time

from message import LoopUntil, Receiver, Sender
from message_factory import (
    GetAdaptioVersion,
    GetSlidesPosition,
    ServiceModeStop,
    ServiceModeTracking,
    StartTracking,
    SetJointGeometry
)


def test_joint_tracking(zmq_sockets, adaptio_app, config_file):
    time.sleep(1)
    request_socket, reply_socket = zmq_sockets

    receive_timeout_ms = 200
    receiver = Receiver(reply_socket, receive_timeout_ms)
    sender = Sender(request_socket)

    # TODO: Find a better solution for this. Maybe attr in Application class.
    cmd = ["--config-file", config_file]

    adaptio_app.run(cmd)

    # Test unstable without a sleep here
    time.sleep(10)

    # Check adaptio started by print adaptio version
    for _ in LoopUntil(15.0):
        sender.send(GetAdaptioVersion())
        version_rsp = receiver.poll_specific("GetAdaptioVersionRsp")
        if version_rsp is not None:
            print("Adaptio version: " + version_rsp["payload"]["version"])
            break
    else:
        assert False, "GetAdaptioVersionRsp not received"

    for _ in LoopUntil(15.0):
        sender.send(SetJointGeometry())
        response_rsp = receiver.poll_specific("SetJointGeometryRsp")
        if response_rsp is not None:
            print("Join Geometry: " + response_rsp["payload"]["result"])
            break
        time.sleep(1)
    else:
        assert False, "SetJointGeometry failed"    

    # Start service mode for joint tracking
    servicemode_tracking_msg = ServiceModeTracking()
    sender.send(servicemode_tracking_msg)

    # Start joint tracking
    start_tracking_msg = StartTracking()
    start_tracking_msg["payload"]["horizontal_offset"] = 5.0
    start_tracking_msg["payload"]["vertical_offset"] = 6.0
    start_tracking_msg["payload"]["joint_tracking_mode"] = 0
    sender.send(start_tracking_msg)
    
    time.sleep(1)

    # Wait for horizontal position < -20
    # Since input image is not adjusted based on new set position
    # it will never converge. Instead the set position will always be increased
    for _ in LoopUntil(10.0):
        sender.send(GetSlidesPosition())
        if receiver.verify(
            "GetSlidesPositionRsp", lambda payload: payload["horizontal"] < -3
        ):
            assert True, "Horizontal position < -3"
            break
        time.sleep(1)
    else:
        assert False, "Horizontal axis not in position"

    # Stop joint tracking
    servicemode_stop = ServiceModeStop()
    sender.send(servicemode_stop)
    time.sleep(1)

    assert True, "Joint tracking test completed"

    adaptio_app.quit(None)
    time.sleep(5)
