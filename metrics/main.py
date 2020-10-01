import json
import logging
import sys

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
            f'{context.server_url}updateExecution', json=post_body)

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


def do_job(scenario):
    # 0. find log file
    log_files = metrics_utils.get_log_files(scenario, context.log_dir)
    if len(log_files) == 0:
        logger.warning(
            f'can not find log file for scenario{scenario} from {context.log_dir}')
        return

    # 1. push simulation log to content-store
    try:
        context.content_set_agent.put_object(log_file_name, log_file_path)
    except Exception as e:
        logger.warning(f'failed to push log file to content-store, err: {e.__str__()}')

    # 2. calculate metrics
    try:
        metric = metrics.do_metric(logger, log_files)
        msg = 'OK'
        success = True
    except Exception as e:
        logger.warning(f'calculate metric failed with err: {e.__str__()}')
        msg = e.__str__()
        success = False
        metric = metrics.failed_metric

    # 3. post result to server
    try:
        on_metrics_result(None, {
            scenario: metric
        }, msg, success)
    except Exception as e:
        logger.warning(
            f'failed to post metric result to server with err: {e.__str__()}')
        return


def main(argv):
    global context
    if metrics_utils.in_sandbox():
        context = metrics_utils.MetricsContext({}, logger)
        scenario = context.scenario
    else:
        scenario = argv[1]

    do_job(scenario)


if __name__ == '__main__':
    main(sys.argv)
