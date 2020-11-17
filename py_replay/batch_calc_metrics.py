import os
import time
import glob
import argparse

from utils import dataset_reader
from utils import metrics_calculator


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-l', type=str)
    args = parser.parse_args()

    if not os.path.exists('./scores'):
        os.mkdir('./scores')
    
    if args.l[-1] == '/':
        args.l = args.l[:-1]

    assert(os.path.isdir(args.l))
    fout = open(os.path.join('scores', args.l) + '.txt', 'w')

    mean_metrics = {}
    tot_no_crash = 0

    for c_id in range(40):
        c = 'config%d' % c_id
        if not os.path.exists(os.path.join(args.l, c)):
            continue
        
        config_file = None
        collision_file = None
        log_file = None

        for s in glob.glob(os.path.join(args.l, c, '*')):
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

        config = dataset_reader.Config(config_file)
        collision = dataset_reader.Collision(collision_file)
        log = dataset_reader.Log(log_file)

        no_crash, metrics = metrics_calculator.calc_metrics(config, log, collision)
        print('#', c, file=fout)
        print('# no_crash', no_crash, file=fout)

        if no_crash:
            score = metrics_calculator.score_of_metrics(metrics)
            tot_no_crash += 1

            for k in metrics:
                if not k in mean_metrics:
                    mean_metrics[k] = []
                
                mean_metrics[k].append(metrics[k])

            print('# metrics', metrics, file=fout)
            print('# score', score, '\n', file=fout)

    for k in mean_metrics:
        mean_metrics[k] = sum(mean_metrics[k]) / len(mean_metrics[k])

    print('# no crash', tot_no_crash, file=fout)
    print('# mean metrics', mean_metrics, file=fout)
    print('# mean score', metrics_calculator.score_of_metrics(mean_metrics), file=fout)
    fout.close()