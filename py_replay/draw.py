import os
import time
import glob
import math
import argparse

import matplotlib.pyplot as plt
from utils import dataset_reader



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-l', type=str)
    args = parser.parse_args()

    assert(os.path.isdir(args.l))

    config_file = None
    collision_file = None
    log_file = None
    pred_file = None

    for s in glob.glob(os.path.join(args.l, '*')):
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
    assert os.path.isfile(pred_file), pred_file

    config = dataset_reader.Config(config_file)
    collision = dataset_reader.Collision(collision_file)
    log = dataset_reader.Log(log_file)
    log.read_prediction(pred_file)

    dataset_file = os.path.join('CSV', config.map, 'vehicle_tracks_%s.csv' % config.csv_id)
    dataset = dataset_reader.Dataset(dataset_file)

    ######################################################################

    for _, track in log.track_dict.items():
        if track.isego == 'no':
            continue

        track_id = track.track_id
        start_time = track.time_stamp_ms_first
        end_time = track.time_stamp_ms_last

        car_descriptor = '%s-%d' % (config.map[-2:], track_id)

        if track_id in config.robot_car_planner:
            car_descriptor += '-%s' % config.robot_car_planner[track_id]
        else:
            car_descriptor += '-replay'
        
        if track.isego == 'yes':
            car_descriptor += '-ego'

        assert track_id in dataset.track_dict

        fig, axs = plt.subplots(1, 2)
        log_v_list = []
        csv_v_list = []

        for t in range(start_time, end_time, dataset_reader.DELTA_TIMESTAMP_MS):
            log_x = track.motion_states[t].x
            log_y = track.motion_states[t].y
            log_v = track.motion_states[t].vx

            csv_x = dataset.track_dict[track_id].motion_states[t + config.StartTimestamp].x - 1000.0
            csv_y = dataset.track_dict[track_id].motion_states[t + config.StartTimestamp].y - 1000.0
            csv_v = math.sqrt(dataset.track_dict[track_id].motion_states[t + config.StartTimestamp].vx ** 2 + dataset.track_dict[track_id].motion_states[t + config.StartTimestamp].vy ** 2)

            #print('log_x: %.3lf, log_y: %.3lf, log_v: %.3lf | csv_x: %.3lf, csv_y: %.3lf csv_v: %.3lf' % (log_x, log_y, log_v, csv_x, csv_y, csv_v))
        
            log_v_list.append(log_v)
            csv_v_list.append(csv_v)
        
        tt = [t * dataset_reader.DELTA_TIMESTAMP_MS / 1000 for t in range(len(log_v_list))]
        axs[0].plot(tt, log_v_list)
        axs[0].set_xlabel('time (s)')
        axs[0].set_ylabel('log velocity (m/s)')
        axs[0].set_title(car_descriptor)
        
        axs[1].plot(tt, csv_v_list)
        axs[1].set_xlabel('time (s)')
        axs[1].set_ylabel('csv velocity (m/s)')
        #axs[1].set_title('%s-csv v' % car_descriptor)

        fig.tight_layout()
        plt.show()


