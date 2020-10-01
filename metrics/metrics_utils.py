import os

from datacenter.contentSetClient import ContentSetClient
from datacenter.labelSetClient import LabelSetClient


def in_sandbox():
    return bool(int(os.getenv("GRAVITI_SANDBOX") or 0))

def get_log_files(scenario, logs_dir) -> []:
    log_files = dict()
    walk_folder(logs_dir, log_files)

    result = []
    for log_file in log_files:
        if log_file.startswith(f'scenario{scenario}'):
            result.append((log_file, log_files[log_file]),)
    return result

class ContentSetAgent(object):
    def __init__(self, content_store_url: str, content_set_id: str, logger):
        self._logger = logger
        self._content_store_url = content_store_url
        self._content_set_id = content_set_id
        self._content_client = ContentSetClient(content_store_url, None)

    def put_object(self, object_name: str, object_path: str):
        with open(object_path, 'rb') as ifile:
            data = ifile.read()
            result = self._content_client.put_object(self._content_set_id, object_name, data)
            if not result:
                raise Exception(f'failed to put object {object_name} to content-store')


def normalize_url(url: str) -> str:
    if url.endswith('/'):
        return url
    else:
        return url + '/'


class MetricsContext(object):
    def __init__(self, config, logger):
        self._logger = logger

        # remote server info
        self._server_url = normalize_url(os.getenv('MODEL_EVALUATION_URL', 'http://localhost:9121'))
        self._content_store_url = normalize_url(os.getenv('CONTENT_STORE_URL', 'http://localhost:9109'))

        # content set for saving simulation log
        self._content_set_id = os.getenv('CONTENT_SET_ID', '')
        if len(self._content_set_id) == 0:
            raise Exception('invalid content-set id for saving simulation logs')

        # job info
        self._log_dir = os.getenv('LOG_DIR', '/tmp')
        self._instance_id = os.getenv('INSTANCE_ID', '9f33ca08-fa4b-4b04-aec0-04371b563db4')
        self._task_tag = os.getenv('TASK_TAG', '9f33ca08-fa4b-4b04-aec0-04371b563db4-predictor')
        self._scenario = os.getenv('SCENARIO_ID', '00')

        # user info
        self._uid = config['UID'] if 'UID' in config else 'user'
        self._gid = config['GID'] if 'GID' in config else 'group'
        self._client_tag = config['CLIENT_TAG'] if 'CLIENT_TAG' in config else '1'
        self._role_code = config['ROLE_CODE'] if 'ROLE_CODE' in config else 'publisher'

        self._content_set_agent = ContentSetAgent(self._content_store_url, self._content_set_id, self._logger)

    @property
    def log_dir(self):
        return os.path.join(self._log_dir, self._task_tag)

    @property
    def scenario(self):
        return self._scenario

    @property
    def server_url(self):
        return self._server_url

    @property
    def task_tag(self):
        return self._task_tag

    @property
    def instance_id(self):
        return self._instance_id

    @property
    def content_set_agent(self):
        return self._content_set_agent


def walk_folder(folder, file_list):
    for root, dirs, files in os.walk(folder):
        for file_entry in files:
            file_list[file_entry] = os.path.join(root, file_entry)
