## READ ME

This folder contains the code which can replay the log files and calculate the metrics.

### Usage

1. copy the log files from `simulator/Log/` to `py_replay/Log` 
2. run `main.py` with `--log` specified.

```bash
# For example
python main.py --log ./Log/config0

# If you want to play the video
python main.py --video --log ./Log/config0
```

### TO-DO

- **modify boundaries, the default values are 0.** 