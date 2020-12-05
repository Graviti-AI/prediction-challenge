import os

from datacenter.contentSetClient import ContentSetClient


def in_sandbox():
    return bool(int(os.getenv('GRAVITI_SANDBOX') or 0))


class LogFile:
    def __init__(self, file_name, file_path):
        self._file_name = file_name
        self._file_path = file_path

    @property
    def name(self):
        return self._file_name

    @property
    def path(self):
        return self._file_path


class ScenarioLogFiles:
    def __init__(self, scenario_id, scenario_name, logs_dir):
        self._config_file = None
        self._collision_file = None
        self._log_file = None

        scenario_name = scenario_name.replace(' ', '_')

        log_files = dict()
        walk_folder(logs_dir, log_files)
        for log_file in log_files:

            if log_file.startswith(f'scenario{scenario_id}_Collision'):
                self._collision_file = LogFile(
                    log_file.replace(f'scenario{scenario_id}_Collision', f'scenario_{scenario_name}_Collision'),
                    log_files[log_file])
            elif log_file.startswith(f'scenario{scenario_id}_test'):
                self._log_file = LogFile(
                    log_file.replace(f'scenario{scenario_id}_test', f'scenario_{scenario_name}_test'),
                    log_files[log_file])
            elif log_file.startswith(f'scenario{scenario_id}_config'):
                self._config_file = LogFile(
                    log_file.replace(f'scenario{scenario_id}_config', f'scenario_{scenario_name}_config'),
                    log_files[log_file])

    @property
    def config_file(self):
        return self._config_file

    @property
    def collision_file(self):
        return self._collision_file

    @property
    def log_file(self):
        return self._log_file


def get_log_files(scenario_id, scenario_name, logs_dir) -> ScenarioLogFiles:
    return ScenarioLogFiles(scenario_id, scenario_name, logs_dir)


class ContentSetAgent(object):
    def __init__(self, content_store_url: str, content_set_id: str, logger):
        self._logger = logger
        self._content_store_url = content_store_url
        self._content_set_id = content_set_id
        # self._auth = Auth()
        self._content_client = ContentSetClient(content_store_url, None)

    def put_object(self, object_name: str, object_path: str, headers=None):
        with open(object_path, 'rb') as ifile:
            data = ifile.read()
            result = self._content_client.put_object(self._content_set_id, object_name, data, headers)
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
        self._scenario_id = os.getenv('SCENARIO_ID', '00')
        self._scenario_name = os.getenv('SCENARIO_NAME', 'unknown')

        # user info
        self._uid = config.get('UID', 'user')
        self._gid = config.get('GID', 'group')
        self._client_tag = config.get('CLIENT_TAG', '1')
        self._role_code = config.get('ROLE_CODE', 'publisher')
        self._user_api_key = config.get('API_KEY', '')
        self._user_x_token = config.get('X_TOKEN', '')

        # content-store for saving simulation logs
        self._content_set_agent = ContentSetAgent(self._content_store_url, self._content_set_id, self._logger)

    def pack_user_info_to_request_header(self, header: {} = None):
        if header is None:
            header = {}
        header['User-Id'] = self._uid
        header['Company-Id'] = self._gid
        header['Role-Code'] = self._role_code
        header['Client-Tag'] = self._client_tag
        header['X-Token'] = self._user_x_token
        header['Api-Key'] = self._user_api_key
        return header

    @property
    def log_dir(self):
        return os.path.join(self._log_dir, self._task_tag)

    @property
    def scenario_id(self):
        return self._scenario_id

    @property
    def scenario_name(self):
        return self._scenario_name

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

    @property
    def uid(self):
        return self._uid

    @property
    def gid(self):
        return self._gid

    @property
    def user_role_code(self):
        return self._role_code

    @property
    def user_client_gat(self):
        return self._client_tag

    @property
    def user_access_token(self):
        return self._user_api_key

    @property
    def user_x_token(self):
        return self._user_x_token


def walk_folder(folder, file_list):
    print(f'begin to walk {folder}')
    for root, dirs, files in os.walk(folder):
        print(f'    root: {root}')
        print(f'    dirs: {dirs}')
        for file_entry in files:
            print(f'        file_entry: {file_entry}')
            file_list[file_entry] = os.path.join(root, file_entry)
