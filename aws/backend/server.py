"""server.py"""

import os
import time
import json
from threading import Lock, Thread
from typing import Dict, List, Union
from pprint import pprint

import git 
import numpy
from git.exc import GitCommandError
from flask import Flask, Response, jsonify, render_template, request
from flask.helpers import make_response
from flask_cors import CORS
from flask_limiter import Limiter
from flask_limiter.util import get_remote_address
from folium import Map, Marker, Popup
from pyproj import Transformer
from pyflann import FLANN
from waitress import serve

from config import config
from constants import *

app: Flask = Flask(
    import_name=__name__, 
    template_folder=os.path.abspath('../frontend/templates'), 
    static_folder=os.path.abspath('../frontend/static')
)
app.config["JSON_AS_ASCII"] = False
app.config["JSON_SORT_KEYS"] = False
app.config['RATELIMIT_HEADERS_ENABLED'] = True
Limiter(app, key_func=get_remote_address, default_limits=[config[CONFIG_RATE_LIMITS]])
CORS(app)

_lock: Lock = Lock()

database: List[Dict[str, Union[str, float]]]
if os.path.exists("../data/database.json"):
    with open("../data/database.json") as f:
        database = json.load(f)
else:
    database = [{
        DATABASE_DESCRIPTION: "説明",
        DATABASE_LATITUDE: 35.1356448,
        DATABASE_LONGITUDE: 136.9760683,
    }]
database_for_flann: List[List[float]]
if os.path.exists("../data/database_for_flann.json"):
    with open("../data/database_for_flann.json") as f:
        database_for_flann = json.load(f)
else:
    database_for_flann = [
        [-95874.87170693283, -17368.887856112502],
    ]

transformer: Transformer = Transformer.from_crs("epsg:4326", "epsg:6675")

global_counter: int = 0

class RenderMap:
    def __init__(
        self,
        interval: float = 30.,
    ) -> None:
        p: Thread = Thread(target=self.run, args=(interval, ), daemon=True)
        p.start()

    def run(self, interval: float) -> None:
        while True:
            map = Map(location=[35.1356448, 136.9760683], zoom_start=16)

            for data in database:
                pprint(data)
                popup = Popup(data[DATABASE_DESCRIPTION], min_width=0, max_width=1000)
                Marker(location=[data[DATABASE_LATITUDE], data[DATABASE_LONGITUDE]], popup=popup).add_to(map)

            print()

            global global_counter
            print(global_counter)
            if global_counter > 0:
                os.remove(os.path.join(os.path.abspath(config[CONFIG_MAP_HTML_PATH]), f"map-{global_counter-1}.html"))
            map.save(os.path.join(os.path.abspath(config[CONFIG_MAP_HTML_PATH]), f"map-{global_counter}.html"))

            global_counter += 1

            time.sleep(interval)


class PushDatabase:
    def __init__(
        self,
        interval: float = 30.,
    ) -> None:
        p: Thread = Thread(target=self.run, args=(interval, ), daemon=True)
        p.start()

        self.repo = git.Repo(os.path.expanduser("~/sony_hackathon"))

    def run(self, interval: float) -> None:
        while True:
            with open(os.path.expanduser('~/sony_hackathon/aws/data/database.json'), 'w') as f:
                json.dump(database, f, indent=4, ensure_ascii=False)

            with open(os.path.expanduser('~/sony_hackathon/aws/data/database_for_flann.json'), 'w') as f:
                json.dump(database_for_flann, f, indent=4, ensure_ascii=False)
            
            try:
                self.repo.git.add(os.path.expanduser('~/sony_hackathon/aws/data/database.json'))
                self.repo.git.commit(os.path.expanduser('~/sony_hackathon/aws/data/database.json'), message='update', author='urasaki')
                self.repo.git.push('origin', 'main')
            except (GitCommandError, AttributeError):
                pass

            try:
                self.repo.git.add(os.path.expanduser('~/sony_hackathon/aws/data/database_for_flann.json'))
                self.repo.git.commit(os.path.expanduser('~/sony_hackathon/aws/data/database_for_flann.json'), message='update', author='urasaki')
                self.repo.git.push('origin', 'main')
            except (GitCommandError, AttributeError):
                pass

            time.sleep(interval)


@app.route('/', methods=["POST"])
def main() -> Response:
    requests: Dict[str, str] = request.get_json(force=True)

    pprint(requests)

    responses: Dict[str, bool] = {
        RESPONSE_IS_INSIDE: False,
    }

    latitude: float = float(requests[REQUEST_LATITUDE])
    longitude: float = float(requests[REQUEST_LONGITUDE])

    x, y = transformer.transform(latitude, longitude)

    if requests[REQUEST_IS_FELL]:
        _lock.acquire()

        data: Dict[str, Union[str, float]] = {
            DATABASE_DESCRIPTION: "説明",
            DATABASE_LATITUDE: latitude,
            DATABASE_LONGITUDE: longitude,
        }
        database.append(data)

        data = [x, y]
        database_for_flann.append(data)

        _lock.release()
    else:
        flann: FLANN = FLANN()
        flann.build_index(numpy.array(database_for_flann))
        _, dists = flann.nn_index(numpy.array([x, y]), num_neighbors=1)

        if dists[0] < config[CONFIG_DANGER_AREA_RANGE]:
            responses[RESPONSE_IS_INSIDE] = True


    return make_response(jsonify(responses))


@app.route('/map')
def index() -> str:
    if global_counter > 0:
        return render_template(f"map-{global_counter-1}.html", title="sony", name="sony")
    else:
        return render_template(f"defalut.html", title="sony", name="sony")


if __name__ == "__main__":
    RenderMap()
    PushDatabase()

    serve(app, host='0.0.0.0', port=5000, threads=1)
