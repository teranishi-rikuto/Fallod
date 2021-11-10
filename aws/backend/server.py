"""server.py"""

import os
import time
from threading import Lock, Thread
from typing import Dict, List, Union

import numpy
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

database: List[Dict[str, Union[str, float]]] = [{
    DATABASE_DESCRIPTION: "説明",
    DATABASE_LATITUDE: 35.1356448,
    DATABASE_LONGITUDE: 136.9760683,
}]
database_for_flann: List[float] = [
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

            from pprint import pprint
            
            for data in database:
                pprint(data)
                popup = Popup(data[DATABASE_DESCRIPTION], min_width=0, max_width=1000)
                Marker(location=[data[DATABASE_LATITUDE], data[DATABASE_LONGITUDE]], popup=popup).add_to(map)

            global global_counter
            print(global_counter)
            if global_counter > 0:
                os.remove(os.path.join(os.path.abspath(config[CONFIG_MAP_HTML_PATH]), f"map-{global_counter-1}.html"))
            map.save(os.path.join(os.path.abspath(config[CONFIG_MAP_HTML_PATH]), f"map-{global_counter}.html"))

            global_counter += 1

            time.sleep(interval)


@app.route('/', methods=["POST"])
def main() -> Response:
    requests: Dict[str, str] = request.get_json(force=True)

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

        print(dists)

        if dists[0] < config[CONFIG_DANGER_AREA_RANGE]:
            responses[RESPONSE_IS_INSIDE] = True


    return make_response(jsonify(responses))


@app.route('/map')
def index() -> Response:
    if global_counter > 0:
        return render_template(f"map-{global_counter-1}.html", title="sony", name="sony")
    else:
        return render_template(f"defalut.html", title="sony", name="sony")


if __name__ == "__main__":
    RenderMap()

    serve(app, host='0.0.0.0', port=5000, threads=30)
