# 同网络下配置
echo "开始创建 Redis 集群"

ip="172.20.2.2:6379 172.20.2.3:6379 172.20.2.4:6379 172.20.2.5:6379 172.20.2.6:6379 172.20.2.7:6379"
echo "host:port is $ip"

docker exec dev_study_redis_11 sh -c "echo yes | redis-cli -a 12345678 --cluster create $ip --cluster-replicas 1"


##############################################################################################################
# 使用这个
# 不同网络下配置
echo "开始创建 Redis 集群"

ip="192.168.9.99"
ips="$ip:9201 $ip:9202 $ip:9203 $ip:9204 $ip:9205 $ip:9206"
echo "host:port is $ips"

docker exec dev_study_redis_9201 sh -c "echo yes | redis-cli -a 12345678 --cluster create $ips --cluster-replicas 1"
echo "结束创建 Redis 集群"
