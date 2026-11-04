"""Run records under the project data root (DESIGN.md §10; file-interfaces skill).

Root: `data_dir` -> $BLUEBOAT_DATA_DIR -> parent of the first $COLCON_PREFIX_PATH entry -> cwd;
the run folder `<root>/data/Replanner_data/<YYYY_MM_DD-HH_MM_SS>-<mission>-<condition>/` is
claimed exclusively (-2, -3, ...); nothing is ever overwritten (CM-7).
"""

from __future__ import annotations

import json
import os


def data_root(data_dir: str | None = None) -> str:
    if data_dir:
        return data_dir
    env = os.environ.get("BLUEBOAT_DATA_DIR")
    if env:
        return env
    cpp = os.environ.get("COLCON_PREFIX_PATH")
    if cpp:
        return os.path.dirname(cpp.split(os.pathsep)[0].rstrip("/"))
    return os.getcwd()


def claim_dir(parent: str, name: str) -> str:
    os.makedirs(parent, exist_ok=True)
    k = 1
    while True:
        cand = os.path.join(parent, name if k == 1 else f"{name}-{k}")
        try:
            os.mkdir(cand)
            return cand
        except FileExistsError:
            k += 1


class RunRecorder:
    def __init__(self, root: str, stamp: str, mission: str, condition: str):
        base = os.path.join(root, "data", "Replanner_data")
        self.dir = claim_dir(base, f"{stamp}-{mission}-{condition}")
        os.mkdir(os.path.join(self.dir, "plans"))
        self._fh = {}

    def write_once(self, name: str, obj) -> str:
        path = os.path.join(self.dir, name)
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
        with os.fdopen(fd, "w") as fh:
            if isinstance(obj, str):
                fh.write(obj)
            else:
                json.dump(obj, fh, indent=1, default=str)
        return path

    def append(self, name: str, record: dict) -> None:
        fh = self._fh.get(name)
        if fh is None:
            fh = open(os.path.join(self.dir, name), "a")
            self._fh[name] = fh
        fh.write(json.dumps(record, default=str) + "\n")
        fh.flush()

    def close(self) -> None:
        for fh in self._fh.values():
            fh.close()
        self._fh = {}
