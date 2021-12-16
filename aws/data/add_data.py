"""add_data.py"""

import csv
import os
import json
from pprint import pprint
from typing import Final

from pyproj import Transformer

DATABASE_DESCRIPTION: Final = "description"
DATABASE_LATITUDE: Final = "latitude"
DATABASE_LONGITUDE: Final = "longitude"

transformer: Transformer = Transformer.from_crs("epsg:4326", "epsg:6675")

def cvt(lat: str, lgn: str):
    lat_list = lat.split('.')
    lat = float(lat_list[0]) + float(lat_list[1]) / 60 + float('.'.join([lat_list[2], lat_list[3]])) / 60 / 60
    lng_list = lgn.split('.')
    lng = float(lng_list[0]) + float(lng_list[1]) / 60 + float('.'.join([lng_list[2], lng_list[3]])) / 60 / 60

    return lat, lng


def main():
    database = [{
        DATABASE_DESCRIPTION: "dummy",
        DATABASE_LATITUDE: 0.0,
        DATABASE_LONGITUDE: 0.0,
    }]
    database_for_flann = [
        [0.0, 0.0],
    ]

    with open('link_info.csv', newline='', encoding='sjis') as f:
        reader = csv.reader(f)
        for i, row in enumerate(reader):
            if i == 0: 
                print(row)
                continue

            if str(row[22]) != "0":
                lat, lng = cvt(str(row[23]), str(row[24]))
                database_ = {
                    DATABASE_DESCRIPTION: "段差あり",
                    DATABASE_LATITUDE: float(lat),
                    DATABASE_LONGITUDE: float(lng),
                }
                database.append(database_)
                database_for_flann.append([transformer.transform(lat, lng)[0], transformer.transform(lat, lng)[1]])

    with open(os.path.expanduser('database.json'), 'w') as f:
        json.dump(database, f, indent=4, ensure_ascii=False)

    with open(os.path.expanduser('database_for_flann.json'), 'w') as f:
        json.dump(database_for_flann, f, indent=4, ensure_ascii=False)

if __name__ == "__main__":
    main()
