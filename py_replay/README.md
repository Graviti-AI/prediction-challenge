## READ ME

This folder contains the code which can replay the log files and calculate the metrics.

### Usage

1. copy the configuration file and log files from `simulator/Log/`, `simulator/conf` to `py_replay/Log` and `py_replay/conf`
2. run `main_visualize_data.py` with `--config`, `--collision` and `--log` specified.

```bash
# For example
python main.py --conf ./Log/scenario_config_Thu_Oct_22_23\:57\:34_2020.txt --collision ./Log/scenario_Collision_test_Thu_Oct_22_23\:57\:35_2020.txt --log ./Log/scenario_test_Thu_Oct_22_23\:57\:35_2020.txt

# If you want to play the video
python main.py --conf ./Log/scenario_config_Thu_Oct_22_23\:57\:34_2020.txt --collision ./Log/scenario_Collision_test_Thu_Oct_22_23\:57\:35_2020.txt --log ./Log/scenario_test_Thu_Oct_22_23\:57\:35_2020.txt --enable_video
```

### TO-DO

- **modify boundaries, the default values are 0.** 