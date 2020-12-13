import argparse
import os
import requests
import json

base_url = 'https://gas.graviti.cn/gateway'


class Auth:
    def __init__(self, gateway_url, username, password):
        self._gateway_url = gateway_url
        if not self._gateway_url.endswith('/'):
            self._gateway_url = self._gateway_url + '/'

        self.token, self.owner_id, self.group_id, self.client_tag = self._get_token_and_owner_id_group_id(
            self._gateway_url, username, password)

        if self.token is None or self.owner_id is None:
            raise Exception(
                'Failed to get token or owner_id from %s' % gateway_url)

    def sign_headers(self, headers):
        if headers is None:
            headers = {}
        headers['X-Token'] = self.token
        return headers

    @staticmethod
    def _get_token_and_owner_id_group_id(gateway_url, user_name, user_password):
        post_headers = {'Content-Type': 'application/json'}
        response = requests.post(f'{gateway_url}user/api/v3/login',
                                 json={
                                     'key': user_name,
                                     'password': user_password
                                 },
                                 headers=post_headers)

        result = json.loads(response.text)
        if not result['success']:
            raise Exception(
                f'get executions from remote server failed, url={base_url}, error={result["message"]}'
            )
        data = result['data']

        return data['Token'], data['userProfile']['userId'], \
            data['userProfile']['companyId'], data['userProfile']['clientTag']


def post_request(auth, path: str, post_body: {}):
    response = requests.post(f'{base_url}{path}',
                             json=post_body,
                             headers=auth.sign_headers(None))

    if not response.text:
        raise Exception('invalid response')

    result = json.loads(response.text)
    if not result['success']:
        raise Exception(
            f'get executions from remote server failed, url={base_url}, error={result["message"]}'
        )
    return result['data']


def update_evaluation_iamge(auth, evaluation_id: str, image: str):
    path = '/model-evaluation/updateEvaluation'
    post_request(auth, path, {
        'evaluationID': evaluation_id,
        'imageURL': image
    })
    print(f'set evaluation iamge to {image}')


def update_scenario():
    pass


def create_scenario(auth, evaluation_id: str, config_file: str):
    path = '/model-evaluation/berkeley/simulation/createSimulationScenario'
    post_request(auth, path, {
        'name': config_file,
        'description': config_file,
        'configurationFile': config_file,
        'evaluationID': evaluation_id
    })


def get_scenarios_from_remote(auth: Auth, evaluation_id: str):
    path = '/model-evaluation/berkeley/simulation/listSimulationScenarios'
    resp = post_request(auth, path, {
        'evaluationId': evaluation_id
    })
    return resp['scenarios']


def find_scenario_files(folder: str):
    return os.listdir(folder)


def find_scenario_files_not_in_remote(scenarios, scenario_files):
    results = []
    for f in scenario_files:
        found = False
        for s in scenarios:
            if f == s['configurationFile']:
                found = True
                break
        if not found:
            results.append(f)
    return results


def sync_sceanrios(auth, evaluation_id: str):
    scenarios = get_scenarios_from_remote(auth, evaluation_id)
    scenario_files = find_scenario_files(os.path.join('..', 'conf'))

    not_in_remote = find_scenario_files_not_in_remote(
        scenarios, scenario_files)

    print(f'found {len(not_in_remote)} new configuration files')
    for f in not_in_remote:
        create_scenario(auth, evaluation_id, f)
    scenarios = get_scenarios_from_remote(auth, evaluation_id)
    if len(scenarios) != len(scenario_files):
        raise Exception('failed to create new scenario')
    print(f'post new configuartion files succeed')


if __name__ == '__main__':
    import sys
    parser = argparse.ArgumentParser(
        description='get submission logs from server')
    parser.add_argument('-u', help='username to login graviti')
    parser.add_argument('-p', help='password to login graviti')
    parser.add_argument('--image', help='simuator image')
    parser.add_argument('--evaluation-id', help='evaluation id to be deployed')

    args = parser.parse_args()

    auth = Auth(base_url, args.u, args.p)
    sync_sceanrios(auth, args.evaluation_id)
    update_evaluation_iamge(auth, args.evaluation_id, args.image)
