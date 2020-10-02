import metrics_utils
import sys
import logging

VeryLargeErrorValue = 10e6
failed_metric = {"collisions": VeryLargeErrorValue}


def do_metric(logger, simulation_log_files):
    for log_file in simulation_log_files:
        logger.warning(
            f'get logfile: filename={log_file[0]}, filepath={log_file[1]}')
    return {
        "collisions": 100
    }


if __name__ == "__main__":
    scenario_id = sys.argv[1]
    simulation_logs_dir = sys.argv[2]

    # find all all log files under simulation_logs_dir, which name like scenario{scenario_id}*.txt
    log_files = metrics_utils.get_log_files(scenario_id, simulation_logs_dir)
    do_metric(logging.getLogger("metrics"), log_files)
