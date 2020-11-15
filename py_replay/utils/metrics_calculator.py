from .dataset_reader import *

# TODO: specify the boundaries

JERK_BOUNDARY = 0.0
VELO_BOUNDARY = 8.0
YAW2LANE_BOUNDARY = 0.0


def calc_metrics(config: Config, log: Log, collision: Collision, verbose=False):
    if (verbose):
        print("########## Calculating Metrics ##########")

    metrics = {
        'jerk': 0,  # the number of times when jerk exceeds jerk limit
        'velo': 0,  # the number of times when speed exceeds speed limit
        #'yaw2lane': 0,  # yaw angle deviating from the lane direction
        'collision_car': 0,  # number of collisions between cars in the simulation period
        'collision_lane': 0,  # number of collisions between car and lane in the simulation period
        'duration': 0,  # time of survival
        'efficiency': 0,  # how efficient the ego car is
        'courtesy': 0,  # how courtesy the ego car is, i.e., impacting other vehicles
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
            metrics['courtesy'] += (-v_cost_targetcar_sim - jerk_targetcar_sim) / len(value.motion_states)

        # NOTE: skip replay car when calculating metrics
        if value.agent_type == 'ReplayCar' or value.isego == 'no':
            continue

        assert value.agent_type == 'BehaveCar' and value.isego == 'yes'
        if (verbose):
            print('# calc ego car (%d) ...' % value.track_id)

        # Old: metrics['duration'] = max(metrics['duration'], (value.time_stamp_ms_last - value.time_stamp_ms_first) // DELTA_TIMESTAMP_MS)
        metrics['duration'] += 1.0 if log.no_crash else 0.0  # TODO: modified by yaofeng
        metrics['efficiency'] = value.s_now / value.s_tot
        #efficiency_flag = False

        for timestamp in value.motion_states.keys():
            ms = value.motion_states[timestamp]
            assert isinstance(ms, MotionState)

            metrics['jerk'] += ms.jerk ** 2
            metrics['velo'] += (min(ms.velo - VELO_BOUNDARY, 0)) ** 2

            #if (ms.x - config.EgoEndPositionX) ** 2 + (ms.y - config.EgoEndPositionY) ** 2 <= 1 and not efficiency_flag:
            #    metrics['efficiency'] = ((config.EndframeTimestamp - config.StartframeTimestamp) - (
            #                timestamp - value.time_stamp_ms_first)) / 1000
            #    efficiency_flag = True

            #if abs(ms.yaw2lane) >= YAW2LANE_BOUNDARY:
            #    metrics['yaw2lane'] += 1
        
        metrics['jerk'] /= len(value.motion_states)
        metrics['velo'] /= len(value.motion_states)

    for ts, value in collision.record_with_car.items():
        metrics['collision_car'] += len(value)
    
    for ts, value in collision.record_with_lane.items():
        metrics['collision_lane'] += len(value)        

    return metrics


def score_of_metrics(metrics):
    return -metrics['jerk'] - metrics['velo'] + metrics['efficiency'] + metrics['courtesy'] - (metrics['collision_car'] > 0) * 100000 - (1 - metrics['duration']) * 100000000
