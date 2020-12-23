# By Yaofeng Sun, Dec. 2020

from .dataset_reader import *


VELO_BOUNDARY = 8.0


def calc_metrics(config: Config, log: Log, collision: Collision, verbose=False) -> (bool, dict):
    """
    Returns:
        [bool, dict]:
            no_crash [bool]: True means the simulator finished normally 
            metrics [dict]: describe each term of the metrics
    """

    if not log.no_crash:
        return (False, {})

    metrics = {
        'efficiency': 0,  # how efficient the ego car is
        'jerk': 0,  # the number of times when jerk exceeds jerk limit
        'velo': 0,  # the number of times when speed exceeds speed limit
        'courtesy': 0,  # how courtesy the ego car is, i.e., impacting other vehicles
        'collision_car': 0,  # number of collisions between cars in the simulation period
    }
    track_dictionary = log.track_dict

    for key, value in track_dictionary.items():
        assert isinstance(value, Track)

        # calc courtesy
        if key in config.TargetRightofWay:
            if (verbose):
                print('# calc `courtesy` for car (%d)' % key)

            v_cost_targetcar_sim = 0
            jerk_targetcar_sim = 0
            for timestamp in value.motion_states.keys():
                ms = value.motion_states[timestamp]
                v_cost_targetcar_sim += (min(ms.velo - VELO_BOUNDARY, 0)) ** 2
                jerk_targetcar_sim += ms.jerk ** 2

            metrics['courtesy'] += math.sqrt(v_cost_targetcar_sim /
                                             len(value.motion_states))
            metrics['courtesy'] += math.sqrt(jerk_targetcar_sim /
                                             len(value.motion_states))

        # NOTE: skip replay car when calculating metrics
        if value.agent_type == 'ReplayCar' or value.isego == 'no':
            continue

        assert value.agent_type == 'BehaveCar' and value.isego == 'yes'
        if (verbose):
            print('# calc ego car (%d) ...' % value.track_id)

        metrics['efficiency'] = min(1.0, value.s_now / value.s_tot)
        metrics['efficiency'] *= 10.0

        for timestamp in value.motion_states.keys():
            ms = value.motion_states[timestamp]
            assert isinstance(ms, MotionState)

            metrics['jerk'] += ms.jerk ** 2
            metrics['velo'] += (min(ms.velo - VELO_BOUNDARY, 0)) ** 2

        metrics['jerk'] /= len(value.motion_states)
        metrics['jerk'] = math.sqrt(metrics['jerk'])

        metrics['velo'] /= len(value.motion_states)
        metrics['velo'] = math.sqrt(metrics['velo'])

    for ts, value in collision.record_with_car.items():
        metrics['collision_car'] += len(value)

    assert metrics['collision_car'] % 2 == 0, metrics['collision_car']
    metrics['collision_car'] //= 2
    metrics['score'] = score_of_metrics(metrics)
    return (True, metrics)


def score_of_metrics(metrics):
    return metrics['efficiency'] - metrics['jerk'] - metrics['velo'] - metrics['courtesy'] - (metrics['collision_car'] > 0) * 100000
