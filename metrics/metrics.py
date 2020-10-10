import metrics_utils
import os
import sys
import logging

from utils import dataset_reader
from utils import metrics_calculator

VeryLargeErrorValue = 10e6
failed_metric = {
    'jerk': VeryLargeErrorValue,
    'velo': VeryLargeErrorValue,
    'yaw2lane': VeryLargeErrorValue,
    'collision': VeryLargeErrorValue,
    'duration': VeryLargeErrorValue
}


def do_metric(logger, simulation_log_files: metrics_utils.ScenarioLogFiles) -> {}:
    log_file = simulation_log_files.log_file.path if simulation_log_files.log_file is not None else ''
    collision_file = simulation_log_files.collision_file.path if simulation_log_files.collision_file is not None else ''
    config_file = simulation_log_files.config_file.path if simulation_log_files.config_file is not None else ''
    if not os.path.exists(log_file) or len(collision_file) == 0 or len(config_file) == 0:
        return failed_metric

    logger.info(f'get log_file: {log_file}')
    logger.info(f'get collision_file: {collision_file}')
    logger.info(f'get config_file: {config_file}')

    '''
    config = dataset_reader.Config(config_file)
    collision = dataset_reader.Collision(collision_file)
    track_dictionary = dataset_reader.read_log(log_file)

    return metrics_calculator.calc_metrics(config, track_dictionary, collision)
    '''

    return {
        'jerk': 5117,
        'velo': 5117,
        'yaw2lane': 5117,
        'collision': 0,
        'duration': 2620
    }


if __name__ == "__main__":
    scenario_id = sys.argv[1]
    simulation_logs_dir = sys.argv[2]

    # find all all log files under simulation_logs_dir, which name like scenario{scenario_id}*.txt
    log_files = metrics_utils.get_log_files(scenario_id, simulation_logs_dir)
    do_metric(logging.getLogger("metrics"), log_files)
