from abc import ABC

from celery.app.task import Task

from extensions.log import logger


class BaseTask(Task, ABC):

    def before_start(self, task_id, args, kwargs):
        """Handler called before the task starts.

        Arguments:
            task_id (str): Unique id of the task to execute.
            args (Tuple): Original arguments for the task to execute.
            kwargs (Dict): Original keyword arguments for the task to execute.

        Returns:
            None: The return value of this handler is ignored.
        """

        logger.debug(f'{task_id} start before_start'.center(60, '*'))
        logger.info(f'task_id: {task_id}, args: {args}, kwargs: {kwargs}')
        logger.debug(f'{task_id} end before_start'.center(60, '*'))

    def on_success(self, retval, task_id, args, kwargs):
        """Success handler.

        Run by the worker if the task executes successfully.

        Arguments:
            retval (Any): The return value of the task.
            task_id (str): Unique id of the executed task.
            args (Tuple): Original arguments for the executed task.
            kwargs (Dict): Original keyword arguments for the executed task.

        Returns:
            None: The return value of this handler is ignored.
        """

        logger.debug(f'{task_id} start success'.center(60, '*'))
        # the next request
        logger.debug(f'{task_id} end success'.center(60, '*'))

    def on_failure(self, exc, task_id, args, kwargs, einfo):
        """Error handler.

        This is run by the worker when the task fails.

        Arguments:
            exc (Exception): The exception raised by the task.
            task_id (str): Unique id of the failed task.
            args (Tuple): Original arguments for the task that failed.
            kwargs (Dict): Original keyword arguments for the task that failed.
            einfo (~billiard.einfo.ExceptionInfo): Exception information.

        Returns:
            None: The return value of this handler is ignored.
        """

        logger.debug(f'{task_id} start on_failure'.center(60, '*'))
        logger.error(f'task_id: {task_id}, exc: {exc}, einfo: {einfo}')
        logger.debug(f'{task_id} end on_failure'.center(60, '*'))

    def after_return(self, status, retval, task_id, args, kwargs, einfo):
        """Handler called after the task returns.

        Arguments:
            status (str): Current task state.
            retval (Any): Task return value/exception.
            task_id (str): Unique id of the task.
            args (Tuple): Original arguments for the task.
            kwargs (Dict): Original keyword arguments for the task.
            einfo (~billiard.einfo.ExceptionInfo): Exception information.

        Returns:
            None: The return value of this handler is ignored.
        """

        logger.debug(f'{task_id} start after_return'.center(60, '*'))
        logger.info(f'task_id: {task_id}, status: {status}, retval: {retval}')
        logger.debug(f'{task_id} end after_return'.center(60, '*'))

    def on_retry(self, exc, task_id, args, kwargs, einfo):
        """Retry handler.

        This is run by the worker when the task is to be retried.

        Arguments:
            exc (Exception): The exception sent to :meth:`retry`.
            task_id (str): Unique id of the retried task.
            args (Tuple): Original arguments for the retried task.
            kwargs (Dict): Original keyword arguments for the retried task.
            einfo (~billiard.einfo.ExceptionInfo): Exception information.

        Returns:
            None: The return value of this handler is ignored.
        """

        logger.debug(f'{task_id} start on_retry'.center(60, '*'))
        logger.warning(f'task_id: {task_id}, exc: {exc}, einfo: {einfo}')
        logger.debug(f'{task_id} end on_retry'.center(60, '*'))
