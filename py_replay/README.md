## READ ME

This folder contains the code which can replay the log files and calculate the metrics.

### Usage

**`main.py`**

1. copy the log files from `simulator/Log/` to `py_replay/LogX` 
2. run `main.py` with `--log` specified.

```bash
# For example
python main.py --log ./Log/config0

# If you want to play the video
python main.py --log ./Log/config0 --video

# If you want to print more
python main.py --log ./Log/config0 --video --verbose
```

**record_video.py**

1. copy the log files from `simulator/Log/` to `py_replay/LogX` 
2. run `main.py` with `-l` specified.

```bash
# For example

python record_video.py -l Log-v5\ \(11.13\)/
```





### TO-DO

- **modify boundaries, the default values are 0.** 