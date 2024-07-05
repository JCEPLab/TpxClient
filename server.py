import subprocess
from time import sleep
import sys
import os

# Work in progress - this doesn't seem to work on some computers
class TpxServer:

    def __init__(self, path:str):
        self._exe_path = path
        self._process = None

    def start(self, autostart=True, kill_existing=False, path_append=None):
        if self._process is not None:
            if not kill_existing:
                return

            self.stop()

        exe_dir, exe_name = os.path.split(self._exe_path)

        env = os.environ.copy()
        if path_append is not None:
            if type(path_append) not in (list, tuple):
                path_append = [path_append,]

            for s in path_append:
                env["PATH"] += ';' + s

        cmd = None
        if autostart:
            cmd = [exe_name, "--autostart"]
        else:
            cmd = exe_name

        cwd = os.getcwd()
        os.chdir(exe_dir)
        self._process = subprocess.Popen(cmd, env=env)
        os.chdir(cwd)

        sleep(1)

    def stop(self):
        self._process.kill()
        self._process.wait()
        self._process = None