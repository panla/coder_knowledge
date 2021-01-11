# Flask 项目日志记录

## logger handle 配置

```python
import logging
from logging.handlers import TimedRotatingFileHandler


def set_logger_handle(app):
    """配置 logger handle"""

    log_level = 'DEBUG'
    logfile_path = './logs/example.log'

    file_handler = TimedRotatingFileHandler(logfile_path, 'midnight')
    file_handler.setLevel(level=log_level)
    file_handler.setFormatter(
        logging.Formatter('[%(asctime)s>] [%(levelname)s] <-%(filename)s-line %(lineno)d>  %(message)s')
    )
    app.logger.addHandler(file_handler)
    app.logger.setLevel(level=log_level)

```