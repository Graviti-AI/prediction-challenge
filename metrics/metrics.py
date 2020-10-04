import metrics_utils
import sys
import logging

from utils import dataset_reader
from utils import metrics_calculator

VeryLargeErrorValue = 10e6
failed_metric = {"collisions": VeryLargeErrorValue}


def do_metric(logger, simulation_log_files):
    for log_file in simulation_log_files:
        logger.warning(
            f'get logfile: filename={log_file[0]}, filepath={log_file[1]}')
        
        ''' TODO:   
        - You need specify `config_file`, `collision_file`, and `log_file`.
        - The metrics will look like:
            - {'jerk': 5117, 'velo': 5117, 'yaw2lane': 5117, 'collision': 0, 'duration': 2620}

        config = dataset_reader.Config(config_file)
        collision = dataset_reader.Collision(collision_file)
        track_dictionary = dataset_reader.read_log(log_file)

        metrics = metrics_calculator.calc_metrics(config, track_dictionary, collision)
        print('\nmetrics', metrics, '\n')
        '''

    return {
        "collisions": 100
    }


if __name__ == "__main__":
    scenario_id = sys.argv[1]
    simulation_logs_dir = sys.argv[2]

    # find all all log files under simulation_logs_dir, which name like scenario{scenario_id}*.txt
    log_files = metrics_utils.get_log_files(scenario_id, simulation_logs_dir)
    do_metric(logging.getLogger("metrics"), log_files)
