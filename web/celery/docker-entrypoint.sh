# 开启 4 个 worker
celery -A server worker -l INFO -c 4

# 带有定时任务
# celery -A server beat -l INFO & celery -A server worker -l INFO -c 4
