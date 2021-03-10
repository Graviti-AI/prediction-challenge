import os
import json
import argparse
import urllib.request



def download_txt(log_link: list, prefix: str):
    url = None

    for x in log_link:
        if x['simulation_name'][:len(prefix)] == prefix:
            assert url is None
            url = x['log_url']

    assert url is not None

    data = ''
    for line in urllib.request.urlopen(url):
        data += line.decode('utf-8')
    
    return data


def save_text(data: list, save_file: str):
    file_ = open(save_file, 'w')
    print(data, file=file_)



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-l', type=str, help='e.g. Log-TNT')
    args = parser.parse_args()

    os.mkdir(os.path.join('Log', args.l))

    with open('log_link.json', 'r') as Fin:
        log_link = json.load(Fin)
        log_link = log_link['data']['logs']
        
        assert len(log_link) == 48 * 4, len(log_link)

    for c_id in range(48):
        c = 'config%d' % c_id
        os.mkdir(os.path.join('Log', args.l, c))

        config_file = download_txt(log_link, 'scenario_test_config_%d_config' % c_id)
        collision_file = download_txt(log_link, 'scenario_test_config_%d_Collision_test' % c_id)
        log_file = download_txt(log_link, 'scenario_test_config_%d_test' % c_id)
        pred_file = download_txt(log_link, 'scenario_test_config_%d_prediction' % c_id)

        save_text(config_file, os.path.join('Log', args.l, c, 'scenario_config.txt'))
        save_text(collision_file, os.path.join('Log', args.l, c, 'scenario_Collision.txt'))
        save_text(log_file, os.path.join('Log', args.l, c, 'scenario_test.txt'))
        save_text(pred_file, os.path.join('Log', args.l, c, 'scenario_prediction.txt'))

