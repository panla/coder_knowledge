# log

## loguru

```python
__all__ = ['logger']

import sys
import os

from loguru import logger

LEVEL = 'INFO'
PATH = ''

os.makedirs(os.path.dirname(os.path.abspath(PATH)), exist_ok=True)

logger.remove()

logger.add(PATH, level=LEVEL.upper(), backtrace=True, diagnose=True, enqueue=True, compression='tar.gz')
logger.add(sys.stdout, level=LEVEL.upper(), backtrace=True, diagnose=True, enqueue=True)

if __name__ == '__main__':
    logger.debug('this is a debug message')
    logger.info('this is an info message')
    logger.error('this is an error message')
```
