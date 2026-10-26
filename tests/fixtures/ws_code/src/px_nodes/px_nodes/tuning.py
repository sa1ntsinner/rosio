import json
import os


class Tuning:
    def __init__(self, values):
        self.gain = values["gain"]
        self.deadband = values["deadband"]


def load_tuning():
    with open(os.environ["PX_TUNING_FILE"]) as handle:
        return Tuning(json.load(handle))
