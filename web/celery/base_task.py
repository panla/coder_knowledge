from celery.app.task import Task
from celery.worker.request import Request

from loguru import logger


class MyRequest(Request):
    def on_timeout(self, soft, timeout):
        return super().on_timeout(soft, timeout)

    def on_failure(self, exc_info, send_failed_event=True, return_ok=False):
        return super().on_failure(exc_info, send_failed_event=send_failed_event, return_ok=return_ok)

    def on_success(self, failed__retval__runtime, **kwargs):
        return super().on_success(failed__retval__runtime, **kwargs)


class MyTask(Task):
    Request = MyRequest


class BaseTask(Task):

    def on_failure(self, exc, task_id, args, kwargs, einfo):
        """当任务失败时，由worker运行"""

        logger.info('on_failure'.center(80, '*'))
        logger.info(f'exc: {exc}')
        logger.info(f'task_id: {task_id}')
        logger.info(f'args: {args}')
        logger.info(f'kwargs: {kwargs}')
        # logger.info('einfo', einfo)

    def on_success(self, retval, task_id, args, kwargs):
        """如果任务成功执行，则由worker运行"""

        logger.info('on_success'.center(80, '*'))
        # logger.info('retval', retval)
        logger.info(f'task_id: {task_id}')
        logger.info(f'args: {args}')
        logger.info(f'kwargs: {kwargs}')

    # def after_return(self, status, retval, task_id, args, kwargs, einfo):
    #     """任务返回后调用的处理程序"""

    #     logger.info('after_return'.center(80, '*'))
    #     logger.info(f'status: {status}')
    #     # logger.info(f'retval: {retval}')
    #     logger.info(f'task_id: {task_id}')
    #     logger.info(f'args: {args}')
    #     logger.info(f'kwargs: {kwargs}')
    #     # logger.info(f'einfo: {einfo}')
