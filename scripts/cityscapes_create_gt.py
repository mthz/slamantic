#
#copyright Matthias Schoerghuber (AIT).
#
import colorsys
import json
import os

import folium
import numpy
import pymap3d as pm
import requests
import argparse
import tqdm

def generate(args):
    dataset_dir = args.dataset

    # input file
    frames_file = os.path.join(dataset_dir, "frames.txt")
    vehicle_folder = os.path.join(dataset_dir, "vehicle")
    timestamp_folder = os.path.join(dataset_dir, "timestamp")

    # output files
    gt_file = os.path.join(dataset_dir, "gt.txt")
    speed_file = os.path.join(dataset_dir, "speed.txt")
    timestamp_to_frame_file = os.path.join(dataset_dir, "timestamp_frame.txt")

    fframes = open(frames_file)

    origin_lat = None
    origin_lon = None
    origin_alt = None

    ax = []
    ay = []
    az = []

    fgt_file = open(gt_file, 'w')
    fspeed_file = open(speed_file, 'w')
    ftimestamp_to_frame_file = open(timestamp_to_frame_file, 'w')

    last_lon = None
    last_lat = None
    last_gps_timestamps = []

    icolor = 0
    color_range = 250.0

    min_filter_d_lon = 0.00003
    min_filter_d_lat = 0.00003

    map = None

    for frame in tqdm.tqdm(fframes):

        vehicle_frame_file = os.path.join(vehicle_folder, frame.rstrip() + "_vehicle.json")
        timestamp_frame_file = os.path.join(timestamp_folder, frame.rstrip() + "_timestamp.txt")

        timestamp = 0
        with open(timestamp_frame_file) as data_file:
            timestamp = float(data_file.readline()) / 1E9

        ftimestamp_to_frame_file.write(f"{timestamp},{frame}")

        with open(vehicle_frame_file) as data_file:
            data = json.load(data_file)
            lat = data["gpsLatitude"]
            lon = data["gpsLongitude"]
            speed = data["speed"]

            fspeed_file.write(f"{timestamp},{speed}\n")

            # continue
            # print(vehicle_frame_file, timestamp, lat, lon)

            # on first frame, set origin
            if not origin_lat:
                origin_lat = lat
                origin_lon = lon
                if not map and args.map:
                    map = folium.Map(location=[origin_lat, origin_lon], zoom_start=18)

            # skip data if it is the same gps coordinate ; we only take it on new measurments (which will be hopefully different witch each measurment)
            # if we would take non edges, timestamp and gps coordinate would be further away
            if numpy.equal(lat, last_lat) and numpy.equal(lon, last_lon):
                continue

            if last_lat and last_lon:
                if numpy.abs(lat - last_lat) < min_filter_d_lat and numpy.abs(lon - last_lon) < min_filter_d_lon:
                    continue

            last_lat = lat
            last_lon = lon

            last_gps_timestamps.append(timestamp)

            # compute median gps update rate
            if len(last_gps_timestamps) > 5:
                gps_t = numpy.median(numpy.diff(numpy.array(last_gps_timestamps)))
                # print("gps_t", gps_t)

            alt = 111

            if args.google_elevation_api:
                r = requests.get(
                    "https://maps.googleapis.com/maps/api/elevation/json?locations=" + str(lat) + "," + str(
                        lon) + "&key=KEY")
                alt = r.json()["results"][0]["elevation"]

            if not origin_alt:
                origin_alt = alt
                forigin = open(os.path.join(dataset_dir, "gpsorigin_filtt_lat_lon.csv"), 'w')
                forigin.write(str(origin_lat) + " " + str(origin_lon) + " " + str(origin_alt))

            icolor = icolor + 1
            if icolor > color_range:
                icolor = 0

            if map:
                rgb = colorsys.hsv_to_rgb(icolor / color_range, 1.0, 1.0)
                rgb_hex = '#%02x%02x%02x' % (int(rgb[0] * 255), int(rgb[1] * 255), int(rgb[2] * 255))
                folium.CircleMarker([lat, lon], fill_color=rgb_hex, stroke=False, radius=5, fill=True, fill_opacity=1,
                                    popup=frame).add_to(map)

            x, y, z = pm.geodetic2enu(lat, lon, alt, origin_lat, origin_lon, origin_alt)
            ax.append(x)
            ay.append(y)
            az.append(z)
            # we turn the coordiante frame so that z looks north
            fgt_file.write(" ".join([str(timestamp), str(x), str(-z), str(y), "0", "0", "0", "1"]) + "\n")

    # mark begin and end
    if map:
        folium.Marker([lat, lon], popup=folium.Popup("end", show=True),
                      icon=folium.Icon(color='red')).add_to(map)
        folium.Marker([origin_lat, origin_lon], icon=folium.Icon(color='green'),
                      popup=folium.Popup("start", show=True), ).add_to(map)
        map.save(os.path.join(dataset_dir,"gps.html"))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='create citycapes ground truth files from gps and vehicle data')

    parser.add_argument("--dataset", help="path to dataset which contains frames.txt", default=".")
    parser.add_argument("--map", action="store_true", help="store gps trajectory as html", default=False)
    parser.add_argument("--google-elevation-api", action="store_true", help="use google elevation api", default=False)

    args = parser.parse_args()

    if not os.path.exists(os.path.join(args.dataset, "frames.txt")):
        raise ValueError("frames.txt does not exist in path. is it the right path?")

    generate(args)

    print ("finsihed")
