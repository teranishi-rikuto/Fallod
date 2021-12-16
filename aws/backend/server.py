"""server.py"""

import os
import time
import json
from threading import Lock, Thread
from typing import Dict, List, Union
from pprint import pprint
from folium.map import Icon

import git 
import numpy
from git.exc import GitCommandError
from flask import Flask, Response, jsonify, render_template, request
from flask.helpers import make_response
from flask_cors import CORS
from flask_limiter import Limiter
from flask_limiter.util import get_remote_address
from folium import Map, Marker, Popup, Circle, LayerControl
from folium.raster_layers import TileLayer
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
        DATABASE_DESCRIPTION: "dummy",
        DATABASE_LATITUDE: 0.0,
        DATABASE_LONGITUDE: 0.0,
    }]

database_for_flann: List[List[float]]
if os.path.exists("../data/database_for_flann.json"):
    with open("../data/database_for_flann.json") as f:
        database_for_flann = json.load(f)
else:
    database_for_flann = [
        [0.0, 0.0],
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
                if data[DATABASE_DESCRIPTION] == "ユーザーが転倒した地点":
                    popup = Popup(data[DATABASE_DESCRIPTION], min_width=0, max_width=1000)
                    Circle(location=[data[DATABASE_LATITUDE], data[DATABASE_LONGITUDE]], radius=config[CONFIG_DANGER_AREA_RANGE], popup=popup, color='#3186cc', fill_color='#3186cc').add_to(map)
                
                else:
                    popup = Popup(data[DATABASE_DESCRIPTION], min_width=0, max_width=1000)
                    icon = Icon(color="orange")
                    Marker(location=[data[DATABASE_LATITUDE], data[DATABASE_LONGITUDE]], popup=popup, icon=icon).add_to(map)

            global global_counter
            if global_counter > 0:
                os.remove(os.path.join(os.path.abspath(config[CONFIG_MAP_HTML_PATH]), f"map-{global_counter-1}.html"))

            tsunami_tile = TileLayer(
                tiles='https://disaportaldata.gsi.go.jp/raster/04_tsunami_newlegend_data/{z}/{x}/{y}.png',
                fmt='image/png',
                attr="津波浸水想定",
                name="津波浸水想定",
                tms=False,
                overlay=True,
                control=True,
                show=False,
                opacity=0.7
            )
            tsunami_tile.add_to(map)

            flood_tile = TileLayer(
                tiles='https://disaportaldata.gsi.go.jp/raster/01_flood_l2_shinsuishin_data/{z}/{x}/{y}.png',
                fmt='image/png',
                attr="洪水浸水想定",
                name="洪水浸水想定",
                tms=False,
                overlay=True,
                control=True,
                show=False,
                opacity=0.7
            )
            flood_tile.add_to(map)

            sediment_tile = TileLayer(
                tiles='https://disaportaldata.gsi.go.jp/raster/05_dosekiryukeikaikuiki/{z}/{x}/{y}.png',
                fmt='image/png',
                attr="土砂災害警戒区域",
                name="土砂災害警戒区域",
                tms=False,
                overlay=True,
                control=True,
                show=False,
                opacity=0.7
            )
            sediment_tile.add_to(map)

            LayerControl().add_to(map)
            map.save(os.path.join(os.path.abspath(config[CONFIG_MAP_HTML_PATH]), f"map.html"))
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
                self.repo.git.add(os.path.expanduser('~/sony_hackathon/aws/data/database_for_flann.json'))
                self.repo.git.commit(os.path.expanduser('~/sony_hackathon/aws/data/database.json'), message='update', author='anonymous')
                self.repo.git.commit(os.path.expanduser('~/sony_hackathon/aws/data/database_for_flann.json'), message='update', author='anonymous')
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
            DATABASE_DESCRIPTION: "ユーザーが転倒した地点",
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
def index_map() -> str:
    if global_counter > 0:
        return render_template(f"wrapper.html", title="fallod", name="fallod")
    else:
        return render_template(f"defalut.html", title="fallod", name="fallod")

@app.route('/map.html')
def render_map() -> str:
    if global_counter > 0:
        return render_template(f"map.html", title="fallod", name="fallod")
    else:
        return render_template(f"defalut.html", title="fallod", name="fallod")

@app.route('/map-raw')
def index_map_raw() -> str:
    if global_counter > 0:
        return render_template(f"map-{global_counter-1}.html", title="fallod", name="fallod")
    else:
        return render_template(f"defalut.html", title="fallod", name="fallod")


if __name__ == "__main__":
    RenderMap()
    # PushDatabase()

    serve(app, host='0.0.0.0', port=5000, threads=4)
