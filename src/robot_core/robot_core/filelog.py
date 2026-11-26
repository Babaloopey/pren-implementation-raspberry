"""Plain-text log files next to the ROS log, one line per event with a wall-clock timestamp."""
import logging
import os
from datetime import datetime


def file_logger(name, directory, prefix):
    """Logger writing to <directory>/<prefix>_<YYYYmmdd_HHMMSS>.log. Returns (logger, path)."""
    directory = os.path.expanduser(directory)
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, f'{prefix}_{datetime.now():%Y%m%d_%H%M%S}.log')
    logger = logging.getLogger(f'robot_core.{name}')
    logger.setLevel(logging.INFO)
    logger.propagate = False
    handler = logging.FileHandler(path, encoding='utf-8')
    handler.setFormatter(logging.Formatter('%(asctime)s.%(msecs)03d  %(message)s', '%H:%M:%S'))
    logger.addHandler(handler)
    return logger, path
