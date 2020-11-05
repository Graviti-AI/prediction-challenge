import os
import time
import glob
import argparse

import matplotlib.pyplot as plt
from matplotlib.widgets import Button

from utils import dataset_reader
from utils import metrics_calculator

from utils import tracks_vis
from utils import map_vis_without_lanelet


def update_plot():
    global fig, timestamp, title_text, track_dictionary, patches_dict, text_dict, axes, collision
    # update text and tracks based on current timestamp
    assert(timestamp <= timestamp_max), "timestamp=%i" % timestamp
    assert(timestamp >= timestamp_min), "timestamp=%i" % timestamp
    assert(timestamp % dataset_reader.DELTA_TIMESTAMP_MS == 0), "timestamp=%i" % timestamp
    title_text.set_text("\nts = {}".format(timestamp))
    tracks_vis.update_objects_plot(timestamp, patches_dict, text_dict, axes,
                                   track_dict=track_dictionary, pedest_dict=None, collision=collision)
    fig.canvas.draw()


def start_playback():
    global timestamp, timestamp_min, timestamp_max, playback_stopped
    playback_stopped = False
    plt.ion()

    step = dataset_reader.DELTA_TIMESTAMP_MS * 10
    while timestamp + step <= timestamp_max and not playback_stopped:
        timestamp += step
        start_time = time.time()
        update_plot()
        end_time = time.time()
        diff_time = end_time - start_time
        plt.pause(max(0.01, step / 1000. - diff_time))
    plt.ioff()


class FrameControlButton(object):
    def __init__(self, position, label):
        self.ax = plt.axes(position)
        self.label = label
        self.button = Button(self.ax, label)
        self.button.on_clicked(self.on_click)

    def on_click(self, event):
        global timestamp, timestamp_min, timestamp_max, playback_stopped

        if self.label == "play":
            if not playback_stopped:
                return
            else:
                start_playback()
                return
        playback_stopped = True
        if self.label == "<<":
            timestamp -= 10*dataset_reader.DELTA_TIMESTAMP_MS
        elif self.label == "<":
            timestamp -= dataset_reader.DELTA_TIMESTAMP_MS
        elif self.label == ">":
            timestamp += dataset_reader.DELTA_TIMESTAMP_MS
        elif self.label == ">>":
            timestamp += 10*dataset_reader.DELTA_TIMESTAMP_MS
        timestamp = min(timestamp, timestamp_max)
        timestamp = max(timestamp, timestamp_min)
        update_plot()



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--log', type=str)
    parser.add_argument('--video', default=False, action='store_true')
    parser.add_argument('--verbose', default=False, action='store_true')
    args = parser.parse_args()

    assert(os.path.isdir(args.log))

    config_file = None
    collision_file = None
    log_file = None

    for s in glob.glob(os.path.join(args.log, '*')):
        t = os.path.split(s)[-1]

        if t.find('Collision') != -1:
            collision_file = s
        elif t.find('config') != -1:
            config_file = s
        else:
            log_file = s

    assert os.path.isfile(config_file), config_file
    assert os.path.isfile(collision_file), collision_file
    assert os.path.isfile(log_file), log_file

    if args.verbose:
        print("########## file info ##########")
        print('# config_file', config_file)
        print('# collision_file', collision_file)
        print('# log_file', log_file)
        print()

    config = dataset_reader.Config(config_file, args.verbose)
    collision = dataset_reader.Collision(collision_file, args.verbose)
    log = dataset_reader.Log(log_file, args.verbose)

    metrics = metrics_calculator.calc_metrics(config, log, collision)

    if not args.video:
        print('Disable Video')
        exit(0)

    # create a figure
    fig, axes = plt.subplots(1, 1)
    fig.canvas.set_window_title("Interaction Dataset Visualization")

    # load and draw the lanelet2 map, either with or without the lanelet2 library
    lat_origin = 0.  # origin is necessary to correctly project the lat lon values in the osm file to the local
    lon_origin = 0.  # coordinates in which the tracks are provided; we decided to use (0|0) for every scenario
    
    maps_file = os.path.join('Maps', 'with_negative_xy', '%s.osm' % config.map)
    map_vis_without_lanelet.draw_map_without_lanelet(maps_file, axes, lat_origin, lon_origin)

    timestamp_min = 1e9
    timestamp_max = 0

    track_dictionary = log.track_dict
    for key, track in track_dictionary.items():
        timestamp_min = min(timestamp_min, track.time_stamp_ms_first)
        timestamp_max = max(timestamp_max, track.time_stamp_ms_last)

    button_pp = FrameControlButton([0.2, 0.05, 0.05, 0.05], '<<')
    button_p = FrameControlButton([0.27, 0.05, 0.05, 0.05], '<')
    button_f = FrameControlButton([0.4, 0.05, 0.05, 0.05], '>')
    button_ff = FrameControlButton([0.47, 0.05, 0.05, 0.05], '>>')

    button_play = FrameControlButton([0.6, 0.05, 0.1, 0.05], 'play')
    button_pause = FrameControlButton([0.71, 0.05, 0.1, 0.05], 'pause')

    # storage for track visualization
    patches_dict = dict()
    text_dict = dict()

    # visualize tracks
    timestamp = timestamp_min
    title_text = fig.suptitle("")
    playback_stopped = True
    update_plot()
    plt.show()
