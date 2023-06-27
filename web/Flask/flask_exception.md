# Flask Log

[TOC]

## code

`apps.utils`

```python
from typing import Any, Tuple

from flask import current_app


def resp_error(status_code: int = 400, msg: str = 'error', print_msg: str = '', code: int = 40000) -> Tuple[dict, int]:

    if print_msg:
        if not msg:
            msg = print_msg
    else:
        if msg and msg != 'error':
            print_msg = msg

    if print_msg:
        current_app.logger.error(print_msg)

    return {'success': False, 'code': code, 'msg': msg, 'data': None}, status_code
```

`exception.py`

```python
import traceback

from flask import current_app
from flask import request

from werkzeug.exceptions import HTTPException

from apps.utils import resp_error

def process_exception(error):
    """全局异常处理"""

    current_app.logger.error('start error'.center(40, '*'))
    current_app.logger.error(f'{request.method}, {request.path}'.center(60, '*'))
    current_app.logger.error(traceback.format_exc())

    if isinstance(error, HTTPException):
        current_app.logger.error('error is {} {} end.'.format(error.code, error.description))
        current_app.logger.error('end error'.center(40, '*'))
        return resp_error(status_code=error.code, msg=error.description)

    current_app.logger.error('error is {} end.'.format(error))
    current_app.logger.error('end error'.center(40, '*'))

    return resp_error(status_code=500, msg=f'{error}')

```

init

```python
app = Flask(__name__)

app.register_error_handler(Exception, process_exception)
```

## 自定义 code

方案一

```text
自定义大量异常，去抛出 捕获异常
```

方案二

```text
只区分正常和不正常
```

方案三

```python
import json

from werkzeug.exceptions import NotFound

# use
raise NotFound(json.dumps({"status_code": 404, "msg": "", "print_msg": "", "code": 40404}))

# catch
description = json.loads(error.description)
return resp_error(**description)

```
