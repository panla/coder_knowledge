# 开启 4 个 worker
celery -A server worker -l INFO -c 4
