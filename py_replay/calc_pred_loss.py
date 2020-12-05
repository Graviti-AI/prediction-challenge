import os
import time
import glob
import argparse
import numpy as np

from utils import dataset_reader
from utils import metrics_calculator


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-l', type=str)
    args = parser.parse_args()

    if not os.path.exists('./pred_loss'):
        os.mkdir('./pred_loss')
    
    if args.l[-1] == '/':
        args.l = args.l[:-1]

    assert(os.path.isdir(args.l))
    fout = open(os.path.join('pred_loss', args.l) + '.txt', 'w')

    mean_behavior_loss = []
    mean_replay_loss = []

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
        assert os.path.isfile(pred_file), pred_file

        config = dataset_reader.Config(config_file)
        collision = dataset_reader.Collision(collision_file)
        log = dataset_reader.Log(log_file)
        log.read_prediction(pred_file)

        ############################

        behavior_loss = []
        replay_loss = []

        for _, track in log.track_dict.items():
            if track.isego == 'yes':
                continue
            
            track_id = track.track_id
            start_time = track.time_stamp_ms_first
            end_time = track.time_stamp_ms_last

            p_loss = []
            for t in range(start_time, end_time, dataset_reader.DELTA_TIMESTAMP_MS):
                if track.motion_states[t].pred_loss is not None:
                    p_loss.append(track.motion_states[t].pred_loss)
            
            if len(p_loss) > 0:
                if track_id in config.robot_car_planner:
                    behavior_loss.append(np.mean(p_loss))
                else:
                    replay_loss.append(np.mean(p_loss))
        
        metrics = {
            'replay': np.mean(replay_loss) if len(replay_loss) > 0 else None,
            'behavior': np.mean(behavior_loss) if len(behavior_loss) > 0 else None
        }

        print('#', c, file=fout)
        print('# loss', metrics, '\n', file=fout)

        mean_behavior_loss.extend(behavior_loss)
        mean_replay_loss.extend(replay_loss)

    mean_metrics = {
        'replay': np.mean(mean_replay_loss),
        'behavior': np.mean(mean_behavior_loss)
    }

    print('\n# mean metrics', mean_metrics, file=fout)
    fout.close()