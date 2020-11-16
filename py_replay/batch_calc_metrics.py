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
    
    assert(os.path.isdir(args.l))
    fout = open(os.path.join('scores', args.l) + '.txt', 'w')

    mean_score = []

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

        metrics = metrics_calculator.calc_metrics(config, log, collision)
        score = metrics_calculator.score_of_metrics(metrics)
        mean_score.append(score)

        print('#', c, file=fout)
        print('# metrics', metrics, file=fout)
        print('# score of metrics', score, '\n', file=fout)

    print('# mean score', sum(mean_score) / len(mean_score), file=fout)
    fout.close()