import os
import time
import glob
import argparse

import matplotlib.pyplot as plt
from matplotlib.widgets import Button

from PIL import Image

from utils import dataset_reader
from utils import metrics_calculator

from utils import tracks_vis
from utils import map_vis_without_lanelet


def update_plot(timestamp, fig, title_text, patches_dict, in_line_dict, ex_line_dict, text_dict, axes):
    global track_dictionary, collision
    # update text and tracks based on current timestamp
    assert(timestamp % dataset_reader.DELTA_TIMESTAMP_MS == 0), "timestamp=%i" % timestamp
    title_text.set_text("\nts = {}".format(timestamp))
    tracks_vis.update_objects_plot(timestamp, patches_dict, in_line_dict, ex_line_dict, text_dict, axes,
                                   track_dict=track_dictionary, pedest_dict=None, collision=collision,verbose=False)
    fig.canvas.draw()


def start_playback(timestamp_min, timestamp_max, c):
    # create a figure
    fig, axes = plt.subplots(1, 1)
    fig.canvas.set_window_title(c)

    # load and draw the lanelet2 map, either with or without the lanelet2 library
    lat_origin = 0.  # origin is necessary to correctly project the lat lon values in the osm file to the local
    lon_origin = 0.  # coordinates in which the tracks are provided; we decided to use (0|0) for every scenario
    
    maps_file = os.path.join('Maps', 'with_negative_xy', '%s.osm' % config.map)
    map_vis_without_lanelet.draw_map_without_lanelet(maps_file, axes, lat_origin, lon_origin)

    # storage for track visualization
    patches_dict = dict()
    in_line_dict = dict()
    ex_line_dict = dict()
    text_dict = dict()

    # visualize tracks
    title_text = fig.suptitle("")
    step = dataset_reader.DELTA_TIMESTAMP_MS

    print("Generate Images %s ..." % c)
    os.mkdir(os.path.join('visualization', args.l, c))

    for timestamp in range(timestamp_min, timestamp_max, step):
        update_plot(timestamp, fig, title_text, patches_dict, in_line_dict, ex_line_dict, text_dict, axes)
        plt.savefig(os.path.join('visualization', args.l, c, '%d.png' % timestamp), bbox_inches='tight')
    plt.close()

    print("Generate Video ...")
    ims = []
    for timestamp in range(timestamp_min, timestamp_max, step):
        ims.append(Image.open(os.path.join('visualization', args.l, c, '%d.png' % timestamp)))

    ims[0].save(os.path.join('visualization', args.l, c + '.gif'), save_all=True, append_images=ims[1:], optimize=False, duration=step, loop=0)
    
    print("Remove folder ...\n")
    for timestamp in range(timestamp_min, timestamp_max, step):
        os.remove(os.path.join('visualization', args.l, c, '%d.png' % timestamp))
    
    os.rmdir(os.path.join('visualization', args.l, c))



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-l', type=str)
    parser.add_argument('--pred', default=False, action='store_true')
    args = parser.parse_args()

    if not os.path.exists('./visualization'):
        os.mkdir('./visualization')
    
    assert(os.path.isdir(args.l))
    os.mkdir(os.path.join('visualization', args.l))

    for c_id in range(48):
        c = 'config%d' % c_id
        if not os.path.exists(os.path.join(args.l, c)):
            continue
            
        config_file = None
        collision_file = None
        log_file = None
        pred_file = None

        for s in glob.glob(os.path.join(args.l, c, '*')):
            t = os.path.split(s)[-1]

            if t.find('pred') != -1:
                pred_file = s
            elif t.find('Collision') != -1:
                collision_file = s
            elif t.find('config') != -1:
                config_file = s
            else:
                assert t.find('test')
                log_file = s

        assert os.path.isfile(config_file), config_file
        assert os.path.isfile(collision_file), collision_file
        assert os.path.isfile(log_file), log_file

        if args.pred:
            assert pred_file is not None
            assert os.path.isfile(pred_file), pred_file

        config = dataset_reader.Config(config_file)
        collision = dataset_reader.Collision(collision_file)
        log = dataset_reader.Log(log_file)

        if args.pred:
            log.read_prediction(pred_file)

        timestamp_min = 1e9
        timestamp_max = 0

        track_dictionary = log.track_dict
        for key, track in track_dictionary.items():
            timestamp_min = min(timestamp_min, track.time_stamp_ms_first)
            timestamp_max = max(timestamp_max, track.time_stamp_ms_last)

        start_playback(timestamp_min, timestamp_max, c)
