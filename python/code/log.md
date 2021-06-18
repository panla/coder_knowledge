# log

## loguru

```python
import sys
import os

from loguru import logger

LEVEL = 'INFO'
LOG_FILE_PATH = ''

os.makedirs(os.path.dirname(os.path.abspath(LOG_FILE_PATH)), exist_ok=True)

logger.remove()

logger.add(LOG_FILE_PATH, level=LEVEL.upper(), backtrace=True, diagnose=True, enqueue=True)
logger.add(sys.stdout, level=LEVEL.upper(), backtrace=True, diagnose=True, enqueue=True)

__all__ = ['logger']

if __name__ == '__main__':
    logger.debug('this is a debug message')
    logger.info('this is an info message')
    logger.error('this is an error message')
```
