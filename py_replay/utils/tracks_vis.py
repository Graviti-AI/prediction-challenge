#!/usr/bin/env python

import matplotlib
import matplotlib.patches
import matplotlib.lines as lines
import matplotlib.transforms
import numpy as np

from .dataset_reader import Track, MotionState


def rotate_around_center(pts, center, yaw):
    return np.dot(pts - center, np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])) + center


def polygon_xy_from_motionstate(ms, width, length):
    assert isinstance(ms, MotionState)
    lowleft = (ms.x - length / 2., ms.y - width / 2.)
    lowright = (ms.x + length / 2., ms.y - width / 2.)
    upright = (ms.x + length / 2., ms.y + width / 2.)
    upleft = (ms.x - length / 2., ms.y + width / 2.)
    return rotate_around_center(np.array([lowleft, lowright, upright, upleft]), np.array([ms.x, ms.y]), yaw=ms.psi_rad)


def polygon_xy_from_motionstate_pedest(ms, width, length):
    assert isinstance(ms, MotionState)
    lowleft = (ms.x - length / 2., ms.y - width / 2.)
    lowright = (ms.x + length / 2., ms.y - width / 2.)
    upright = (ms.x + length / 2., ms.y + width / 2.)
    upleft = (ms.x - length / 2., ms.y + width / 2.)
    return np.array([lowleft, lowright, upright, upleft])


def update_objects_plot(timestamp, patches_dict, in_line_dict, ex_line_dict, text_dict, axes, track_dict=None, pedest_dict=None, collision=None,verbose=True):
    if verbose:
        print('\n============= timestamp: %d =============' % (timestamp))

    if track_dict is not None:

        for key, value in track_dict.items():
            assert isinstance(value, Track)
            if value.time_stamp_ms_first <= timestamp <= value.time_stamp_ms_last:
                # object is visible
                ms = value.motion_states[timestamp]
                assert isinstance(ms, MotionState)

                if verbose:
                    print('# %d, %s | %s ' % (value.track_id, value.agent_type, ms)) 

                # draw prediction
                if key in in_line_dict:
                    in_line_dict[key].remove()
                    in_line_dict.pop(key)

                if key in ex_line_dict:
                    ex_line_dict[key].remove()
                    ex_line_dict.pop(key)

                # plot prediction trajectories
                if (ms.in_pred is not None) and (ms.ex_pred is not None):
                    if value.isego == 'yes':
                        in_pred_line = lines.Line2D(ms.in_pred[:, 0], ms.in_pred[:, 1], linewidth=1, color='green', zorder=2, axes = axes)                    
                        in_line_dict[key] = in_pred_line
                        axes.add_line(in_pred_line)
                    else:
                        in_pred_line = lines.Line2D(ms.in_pred[:, 0], ms.in_pred[:, 1], linewidth=1, color='red', zorder=2, axes = axes)                    
                        in_line_dict[key] = in_pred_line
                        axes.add_line(in_pred_line)

                        ex_pred_line = lines.Line2D(ms.ex_pred[:, 0], ms.ex_pred[:, 1], linewidth=1, color='purple', zorder=2, axes = axes)                    
                        ex_line_dict[key] = ex_pred_line
                        axes.add_line(ex_pred_line)
                
                #############################

                if key not in patches_dict:
                    width = value.width
                    length = value.length

                    rect = matplotlib.patches.Polygon(polygon_xy_from_motionstate(ms, width, length), closed=True,
                                                      zorder=20)
                    patches_dict[key] = rect
                    axes.add_patch(rect)
                    text_dict[key] = axes.text(ms.x, ms.y + 2, str(key), horizontalalignment='center', zorder=30)
                else:
                    width = value.width
                    length = value.length
                    patches_dict[key].set_xy(polygon_xy_from_motionstate(ms, width, length))
                    text_dict[key].set_position((ms.x, ms.y + 2))
            else:
                if key in patches_dict:
                    patches_dict[key].remove()
                    patches_dict.pop(key)
                    text_dict[key].remove()
                    text_dict.pop(key)
                
                # remove prediction
                if key in in_line_dict:
                    in_line_dict[key].remove()
                    in_line_dict.pop(key)

                if key in ex_line_dict:
                    ex_line_dict[key].remove()
                    ex_line_dict.pop(key)
        
        if timestamp in collision.record_with_car:
            for c in collision.record_with_car[timestamp]:
                if verbose:
                    print('collision_with_car', c)
        
        if timestamp in collision.record_with_lane:
            for c in collision.record_with_lane[timestamp]:
                if verbose:
                    print('collision_with_lane', c)

    if pedest_dict is not None:

        for key, value in pedest_dict.items():
            assert isinstance(value, Track)
            if value.time_stamp_ms_first <= timestamp <= value.time_stamp_ms_last:
                # object is visible
                ms = value.motion_states[timestamp]
                assert isinstance(ms, MotionState)

                if key not in patches_dict:
                    width = 1.5
                    length = 1.5

                    rect = matplotlib.patches.Polygon(polygon_xy_from_motionstate_pedest(ms, width, length), closed=True,
                                                      zorder=20, color='red')
                    patches_dict[key] = rect
                    axes.add_patch(rect)
                    text_dict[key] = axes.text(ms.x, ms.y + 2, str(key), horizontalalignment='center', zorder=30)
                else:
                    width = 1.5
                    length = 1.5
                    patches_dict[key].set_xy(polygon_xy_from_motionstate_pedest(ms, width, length))
                    text_dict[key].set_position((ms.x, ms.y + 2))
            else:
                if key in patches_dict:
                    patches_dict[key].remove()
                    patches_dict.pop(key)
                    text_dict[key].remove()
                    text_dict.pop(key)

