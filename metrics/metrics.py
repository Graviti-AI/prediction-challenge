import metrics_utils
import os
import sys
import logging

from utils import dataset_reader
from utils import metrics_calculator

VeryLargeErrorValue = 10e9
failed_metric = {
    'score': -VeryLargeErrorValue
}


def do_metric(logger, simulation_log_files: metrics_utils.ScenarioLogFiles) -> {}:
    if simulation_log_files.log_file:
        log_file = simulation_log_files.log_file.path
    else:
        logger.error('no log file to calculate metrics')
        return failed_metric

    if simulation_log_files.collision_file:
        collision_file = simulation_log_files.collision_file.path
    else:
        logger.error('no collision file to calculate metrics')
        return failed_metric

    if simulation_log_files.config_file:
        config_file = simulation_log_files.config_file.path
    else:
        logger.error('no config file to calculate metrics')
        return failed_metric

    if not os.path.exists(log_file) or len(collision_file) == 0 or len(config_file) == 0:
        return failed_metric

    logger.info(f'get log_file: {log_file}')
    logger.info(f'get collision_file: {collision_file}')
    logger.info(f'get config_file: {config_file}')

    config = dataset_reader.Config(config_file)
    collision = dataset_reader.Collision(collision_file)
    log = dataset_reader.Log(log_file)

    no_crash, result = metrics_calculator.calc_metrics(config, log, collision)
    
    if not no_crash:
        return failed_metric

    score = metrics_calculator.score_of_metrics(result)
    result['score'] = int(score)
    return result


def main():
    scenario_id = sys.argv[1]
    simulation_logs_dir = sys.argv[2]
    logger = logging.getLogger("metrics")
    # find all all log files under simulation_logs_dir, which name like scenario{scenario_id}*.txt
    log_files = metrics_utils.get_log_files(scenario_id, simulation_logs_dir)
    metrics = do_metric(logger, log_files)
    logger.info(metrics)


if __name__ == "__main__":
    main()
