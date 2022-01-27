echo "开始创建 Redis 集群"

ip="172.20.2.2:6379 172.20.2.3:6379 172.20.2.4:6379 172.20.2.5:6379 172.20.2.6:6379 172.20.2.7:6379"
echo "host:port is $ip"

docker exec dev_study_redis_11 sh -c "echo yes | redis-cli -a 12345678 --cluster create $ip --cluster-replicas 1"