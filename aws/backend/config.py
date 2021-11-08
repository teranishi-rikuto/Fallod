"""config.py"""


from typing import Dict, Union

from constants import *

config: Dict[str, Union[str, int]] = {
    CONFIG_RATE_LIMITS: "1000 per minute",
    CONFIG_DANGER_AREA_RANGE: 10,
    CONFIG_MAP_HTML_PATH: "../frontend/templates",
}

