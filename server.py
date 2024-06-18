import subprocess
from time import sleep

# Work in progress - this doesn't seem to work well at the moment
class TpxServer:

    def __init__(self, path:str):
        self._exe_path = path
        self._process = None

    def start(self, autostart=True, kill_existing=False):
        if self._process is not None:
            if not kill_existing:
                return

            self.stop()

        cmd = None
        if autostart:
            cmd = [self._exe_path, "--autostart"]
        else:
            cmd = self._exe_path

        self._process = subprocess.Popen(cmd, stdout=subprocess.STDOUT, shell=True)

        sleep(1)

    def stop(self):
        self._process.kill()
        self._process.wait()
        self._process = None