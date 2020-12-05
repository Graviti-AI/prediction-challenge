## READ ME

This folder contains the code which can replay the log files and calculate the metrics.

### Usage

1. copy the log files from `simulator/Log/` to `py_replay/LogX` 
2. You can run **`main.py`**, **`record_video.py`**, or **`batch_calc_metrics.py`**

**`main.py`**

```bash
# For example
python main.py --log ./Log/config0

# If you want to play the video
python main.py --log ./Log/config0 --video

# If you want to print more
python main.py --log ./Log/config0 --video --verbose
```

**`record_video.py`**

```bash
# batch generate all the videos
python record_video.py -l Log-v5\ \(11.13\)/
```

**`batch_calc_metrics.py`**

```bash
# batch generate all the scores
python batch_calc_metrics.py -l Log-v6\(11.14\)
```