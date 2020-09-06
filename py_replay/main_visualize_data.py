import os
import time
import argparse

import matplotlib.pyplot as plt
from matplotlib.widgets import Button

from utils import dataset_reader
from utils import dataset_types
from utils import tracks_vis
from utils import map_vis_without_lanelet


def update_plot():
    global fig, timestamp, title_text, track_dictionary, patches_dict, text_dict, axes
    # update text and tracks based on current timestamp
    assert(timestamp <= timestamp_max), "timestamp=%i" % timestamp
    assert(timestamp >= timestamp_min), "timestamp=%i" % timestamp
    assert(timestamp % dataset_types.DELTA_TIMESTAMP_MS == 0), "timestamp=%i" % timestamp
    title_text.set_text("\nts = {}".format(timestamp))
    tracks_vis.update_objects_plot(timestamp, patches_dict, text_dict, axes,
                                   track_dict=track_dictionary, pedest_dict=None)
    fig.canvas.draw()


def start_playback():
    global timestamp, timestamp_min, timestamp_max, playback_stopped
    playback_stopped = False
    plt.ion()
    while timestamp < timestamp_max and not playback_stopped:
        timestamp += dataset_types.DELTA_TIMESTAMP_MS
        start_time = time.time()
        update_plot()
        end_time = time.time()
        diff_time = end_time - start_time
        plt.pause(max(0.001, dataset_types.DELTA_TIMESTAMP_MS / 1000. - diff_time))
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
            timestamp -= 10*dataset_types.DELTA_TIMESTAMP_MS
        elif self.label == "<":
            timestamp -= dataset_types.DELTA_TIMESTAMP_MS
        elif self.label == ">":
            timestamp += dataset_types.DELTA_TIMESTAMP_MS
        elif self.label == ">>":
            timestamp += 10*dataset_types.DELTA_TIMESTAMP_MS
        timestamp = min(timestamp, timestamp_max)
        timestamp = max(timestamp, timestamp_min)
        update_plot()



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, help='config file, e.g. config')
    parser.add_argument('--log', type=str, help='log file, e.g. Sat\ Sep\ \ 5\ 22:26:38\ 2020')
    args = parser.parse_args()

    config_file = os.path.join('Log', '%s.txt' % args.config)
    collision_file = os.path.join('Log', 'Collision_test_%s.txt' % args.log)
    log_file = os.path.join('Log', 'test_%s.txt' % args.log)

    assert os.path.isfile(config_file), config_file
    assert os.path.isfile(collision_file), collision_file
    assert os.path.isfile(log_file), log_file

    print('config_file', config_file)
    print('collision_file', collision_file)
    print('log_file', log_file)

    config = dataset_reader.read_config(config_file)
    maps_file = os.path.join('Maps', 'with_negative_xy', '%s.osm' % config.map)
    
    # create a figure
    fig, axes = plt.subplots(1, 1)
    fig.canvas.set_window_title("Interaction Dataset Visualization")

    # load and draw the lanelet2 map, either with or without the lanelet2 library
    lat_origin = 0.  # origin is necessary to correctly project the lat lon values in the osm file to the local
    lon_origin = 0.  # coordinates in which the tracks are provided; we decided to use (0|0) for every scenario
    print("Loading map...")
    map_vis_without_lanelet.draw_map_without_lanelet(maps_file, axes, lat_origin, lon_origin)

    # load the tracks
    print("Loading tracks...")
    track_dictionary = dataset_reader.read_log(log_file)

    timestamp_min = 1e9
    timestamp_max = 0

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
    print("Plotting...")
    timestamp = timestamp_min
    title_text = fig.suptitle("")
    playback_stopped = True
    update_plot()
    plt.show()
