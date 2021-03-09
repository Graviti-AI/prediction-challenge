import time
import json
import logging
import sys
import traceback
import requests

import metrics
import metrics_utils

logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("simulation-metrics")

context = None


def post_result(result_str: str):
    logger.info(result_str)
    if metrics_utils.in_sandbox():
        post_body = {
            'instanceID': context.instance_id,
            'metrics': result_str
        }
        response = requests.post(
            f'{context.server_url}updateExecution', json=post_body, headers=context.pack_user_info_to_request_header())

        response_body = response.text
        result = json.loads(response_body)
        if not result['success']:
            logger.warning(
                f'failed to post result to remote server, err: {result["message"]}')
    else:
        logger.info(result_str)


def on_metrics_result(total_result, class_result, message, success):
    result_str = json.dumps({
        'overallResults': {
            'totalResult': total_result,
            "classResult": class_result,
        },
        'message': message,
        'success': success
    })
    post_result(result_str)


def upload_log_file(log_file: metrics_utils.LogFile):
    try:
        context.content_set_agent.put_object(
            log_file.name, log_file.path, context.pack_user_info_to_request_header())
    except Exception as e:
        logger.warning(
            f'failed to push log file to content-store, err: {e.__str__()}')


def do_job(scenario_id, scenario_name, update_execution_if_failed):
    try:
        # 0. find log file
        log_files = metrics_utils.get_log_files(
            scenario_id, scenario_name, context.log_dir)
        if log_files.log_file is None or log_files.collision_file is None:
            raise Exception(
                f'can not find log file for scenario{scenario_id} from {context.log_dir}')

        # 1. push simulation log to content-store
        if metrics_utils.in_sandbox():
            upload_log_file(log_files.log_file)
            upload_log_file(log_files.collision_file)
            upload_log_file(log_files.prediction_file)
            upload_log_file(log_files.config_file)

        # 2. calculate metrics
        metric = metrics.do_metric(logger, log_files)
        msg = 'OK'
        success = True
    except Exception as e:
        traceback.print_exc()
        metric = metrics.failed_metric
        msg = e.__str__()
        success = False

    # 3. post result to server
    if success or update_execution_if_failed:
        try:
            on_metrics_result(None, {
                scenario_id: metric
            }, msg, success)
        except Exception as e:
            traceback.print_exc()
            logger.warning(
                f'failed to post metric result to server with err: {e.__str__()}')
            success = False
    return success


def main(argv):
    global context
    if metrics_utils.in_sandbox():
        context = metrics_utils.MetricsContext({}, logger)
        scenario_id = context.scenario_id
        scenario_name = context.scenario_name
    else:
        scenario_id = argv[1]
        scenario_name = argv[2]

    retry_times = 0
    max_try = 3
    retry = True
    while retry:
        success = do_job(scenario_id, scenario_name, not retry)
        retry = not success and retry_times < max_try
        retry_times = retry_times + 1
        time.sleep(5)
        logger.info("calculation failed, try again")

    if not success:
        raise Exception(f'metrics calculation failed')


if __name__ == '__main__':
    main(sys.argv)
