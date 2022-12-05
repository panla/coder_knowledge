
__all__ = ['logger']

import os
import sys

from loguru import logger

import config


os.makedirs(os.path.dirname(os.path.abspath(config.PATH)), exist_ok=True)

logger.remove()

logger.add(config.PATH, level=config.LEVEL.upper(), rotation="00:00", backtrace=True, diagnose=True, enqueue=True)
logger.add(sys.stdout, level=config.LEVEL.upper(), backtrace=True, diagnose=True, enqueue=True)
