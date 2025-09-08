import signal
import subprocess as sb


# sut_start=false means that the software under test is started
# manually, typically via debugger

class Application:

    def __init__(self, path: str, verbosity: str, sut_start: bool):
        self.path = path
        self.verbosity = verbosity
        self.sut_start = sut_start
        self.proc: sb.Popen | None = None

    def run(self, args: list[str]):
        if not self.sut_start:
            print("Run sut manually, with args: {}".format(args))
            return
        command = [self.path, self.verbosity] + args
        print("Starting application: {}".format(command))
        self.proc = sb.Popen(args=command, shell=False)

    def quit(self, code: int | None) -> int:
        if not self.sut_start:
            return -1
        if self.proc is not None:
            self.proc.send_signal(signal.SIGINT)
            return_code = self.proc.wait()

            if return_code == 0 and code is not None:
                return code
            else:
                return return_code

        else:
            return -1

    def kill(self):
        if not self.sut_start:
            return
        if self.proc is not None:
            self.proc.kill()
            self.proc = None
