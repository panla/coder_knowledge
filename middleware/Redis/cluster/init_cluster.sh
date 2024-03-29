# 服务和客户端在同一台服务器下配置
echo "开始创建 Redis 集群"

ip="172.20.2.2:6379 172.20.2.3:6379 172.20.2.4:6379 172.20.2.5:6379 172.20.2.6:6379 172.20.2.7:6379"
echo "host:port is $ip"

docker exec dev_study_redis_11 sh -c "echo yes | redis-cli -a 12345678 --cluster create $ip --cluster-replicas 1"



##############################################################################################################
# 服务与客户端在不同服务器，服务在 192.168.9.99 客户端在 192.168.9.96
# 使用这个，
echo "开始创建 Redis 集群"

ip="192.168.9.99"
ips="$ip:9101 $ip:9102 $ip:9103 $ip:9104 $ip:9105 $ip:9106"
echo "host:port is $ips"

docker exec redis-cluster-node-9101 sh -c "echo yes | redis-cli -a 12345678 --cluster create $ips --cluster-replicas 1"
echo "结束创建 Redis 集群"



##############################################################################################################
# TLS

docker exec redis-cluster-node-9101 sh -c "echo yes | redis-cli -a 12345678 --tls --cacert --cert --key --cluster create $ips --cluster-replicas 1"
echo "结束创建 Redis 集群"
