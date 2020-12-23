# READ ME

The `py_replay` folder includes some useful tools for visualization and debugging. You can copy log files from the `simulator` folder and put them into `py_replay/Log/` (see examples), then you have 5 python scripts to use.

### Usage of `main.py`

```bash
python main.py -l [the log file] [--video] [--pred] [--verbose]
```

- You must specify `-l`. E.g., `python main.py -l Log/Log-LSTM/config0/`
- `--video` means displaying the visualization window.
- `--pred` means plotting the prediction trajectories on the visualization window.
- `--verbose` means outputting more information on the console.

### Usage of `calc_metrics.py`

```bash
python calc_metrics.py -l [the log folder]
```

- You must specify `-l`. Note that the `-l` here means the log folder. E.g., `python calc_metrics.py -l Log/Log-LSTM/`.
- This script can create a folder `scores` and save the metrics into that folder.

### Usage of `calc_pred_loss.py`

```bash
python calc_pred_loss.py -l [the log folder]
```

- You must specify `-l`. Note that the `-l` here means the log folder. E.g., `python calc_pred_loss.py -l Log/Log-LSTM/`.
- This script can create a folder `pred_loss` and save the prediction performance into that folder.

### Usage of `record_video.py`

```bash
python record_video.py -l [the log folder] [--pred]
```

- You must specify `-l`. Note that the `-l` here means the log folder. E.g., `python record_video.py -l Log/Log-LSTM/`.
- `--pred` means plotting the prediction trajectories in the videos.
- This script can generate videos for all the log files from that log folder.

### Usage of `plot_speed_profiles.py`

```bash
python plot_speed_profiles.py -l [the log folder]
```

- You must specify `-l`. Note that the `-l` here means the log folder. E.g., `python plot_speed_profiles.py -l Log/Log-LSTM/`.
- This script can generate speed profiles for all the log files from that log folder.

