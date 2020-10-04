from .dataset_reader import *


#TODO: specify the boundary

JERK_BOUNDARY = 0.0
VELO_BOUNDARY = 0.0
YAW2LANE_BOUNDARY  = 0.0



def calc_metrics(config, track_dictionary, collision):
    metrics = {
        'jerk' : 0,
        'velo' : 0,
        'yaw2lane' : 0,
        'collision' : 0,
        'duration': 0,
    }

    for key, value in track_dictionary.items():
        assert isinstance(value, Track)

        # NOTE: skip replay car when calculating metrics
        if value.agent_type == 'ReplayCar':
            continue
        
        print('\n# calc score for car %d ...' % value.track_id)
        metrics['duration'] = max(metrics['duration'], (value.time_stamp_ms_last - value.time_stamp_ms_first) // DELTA_TIMESTAMP_MS)
        
        for timestamp in value.motion_states.keys():
            ms = value.motion_states[timestamp]
            assert isinstance(ms, MotionState)

            #TODO: change the way to calculate metrics.
            if abs(ms.jerk) >= JERK_BOUNDARY:
                metrics['jerk'] += 1
            
            if ms.velo >= VELO_BOUNDARY:
                metrics['velo'] += 1
            
            if abs(ms.yaw2lane) >= YAW2LANE_BOUNDARY:
                metrics['yaw2lane'] += 1

    for ts, value in collision.record.items():
        metrics['collision'] += len(value)
    
    return metrics
