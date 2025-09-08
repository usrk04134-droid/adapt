"""Configuration for pytest."""
import pytest
import zmq
from pathlib import Path
from application import Application


def pytest_addoption(parser):
    """
    This built-in method defines custom options for pytest for adaptio testing
    needs.
    """
    parser.addoption("--path",
                     help="Path to SUT application binary",
                     required=True)

    verbosity_choices = ["silent", "debug", "info", "trace"]
    parser.addoption("--app-verbosity",
                     action="store",
                     default="debug",
                     choices=verbosity_choices,
                     help=f"Set verbosity level: {verbosity_choices}")

    parser.addoption("--sut-start",
                     action="store_true",
                     default=True,
                     help="If True: SUT is started automatically. "
                          "If False: SUT requires manual start")

    config_choices = ['sil', 'hil', 'lab']
    parser.addoption("--run-type",
                     action="store",
                     default="sil",
                     choices=["sil", "hil", 'lab'],
                     help=f"Specify the run type: {config_choices}."
                          f"Pytest will then look for 'configuration.yaml' in"
                          f"the directory of the run type and configure ip "
                          f"addresses based on run type (TODO).")


@pytest.fixture
def adaptio_app(request) -> Application:
    """Adaptio application fixture.

    :param request: pytest built-in to use callback addoptions
    :return: Application class for adaptio based on path
    """
    verbosity = request.config.getoption("--app-verbosity")
    verbosity = '--' + verbosity
    app = Application(request.config.getoption("--path"),
                      verbosity,
                      request.config.getoption("--sut-start"))

    if "debug" in request.config.getoption("--path"):
        app.disable_doctests_flag = True
    else:
        app.disable_doctests_flag = False

    yield app

    app.quit(None)
    app.kill()


@pytest.fixture
def zmq_context() -> zmq.Context:
    """Defines ZMQ context variable for sockets.

    :return: ZMQ context variable
    """
    context = zmq.Context()
    yield context
    context.term()


@pytest.fixture
def zmq_sockets(request, zmq_context) -> zmq.Socket:
    """Create sockets based on zmq context variable.

    :param request: pytest built-in to use callback addoptions
    :param zmq_context
    :return: zmq sockets
    """

    run_type = request.config.getoption("--run-type")
    if run_type == "sil":
        ip_addr = "tcp://localhost"
    elif run_type == "hil":
        raise NotImplemented
    elif run_type == "lab":
        raise NotImplemented

    request_socket = zmq_context.socket(zmq.PUB)
    request_socket.connect(f"{ip_addr}:5555")
    reply_socket = zmq_context.socket(zmq.SUB)
    reply_socket.subscribe("adaptio_io")
    reply_socket.connect(f"{ip_addr}:5556")
    yield request_socket, reply_socket
    reply_socket.close()
    request_socket.close()


@pytest.fixture(scope='session')
def config_file(request):
    """Find 'configuration.yaml' depending on type of run.

    :param request: pytest built-in to use callback addoptions
    :return: Path for the configuration.yaml file
    """
    file_path = (Path(request.config.getoption("--run-type")).
                 joinpath("configuration.yaml"))
    base_path = Path(__file__).parent.joinpath("configs", file_path)
    if not base_path.exists():
        pytest.fail(f"Config file at path {base_path} doesn't exist")
    return base_path
