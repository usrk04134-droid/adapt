import time
from message import Receiver, Sender, LoopUntil
from message_factory import (SetSlidesPosition,
                             ServiceModeKinematicsControl,
                             GetSlidesPosition, ServiceModeStop, GetAdaptioVersion)




def test_kinematics(zmq_sockets, adaptio_app, config_file):
    # the following testprint function can be removed once/if system tests become
    # stable in the pipeline
    def testprint(x, y):
        print("Test received position:")
        print(x, y)
        return True

    time.sleep(1)
    X_NEG = -30.0
    Z_POS = 20.0

    request_socket, reply_socket = zmq_sockets

    # a "low" value of receive_timeout_ms such as 10 can cause multiple Get requests
    # to be sent before receiving Rsp for unknown reasons. In this test a Get request
    # involves internal messages between the main and controller thread. If the controller
    # thread is blocked for a while such delays could be the result. 
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
        time.sleep(1)
    else:
        assert False, "GetAdaptioVersion not received"

    servicemode_request = ServiceModeKinematicsControl()
    sender.send(servicemode_request)

    set_position_request = SetSlidesPosition()
    set_position_request["payload"]["horizontal"] = X_NEG
    set_position_request["payload"]["vertical"] = Z_POS
    sender.send(set_position_request)

    time.sleep(1)

    for _ in LoopUntil(15.0):
        sender.send(GetSlidesPosition())
        if receiver.verify("GetSlidesPositionRsp",
                           lambda payload: testprint(payload["horizontal"],payload["vertical"]) and 
                           payload["horizontal"] == X_NEG and
                           payload["vertical"] == Z_POS):
            assert True, "Horizontal negative movement"
            break
        time.sleep(1)
    else:
        assert False, "Horizontal positive movement"

    servicemode_stop = ServiceModeStop()
    sender.send(servicemode_stop)

    time.sleep(1)
    assert True, "Kinematics test completed"

    adaptio_app.quit(None)
    time.sleep(5)
