#   本 emqx@172.25.0.4
# 其他 emqx@172.25.0.5

# 加入到目标集群
./bin/emqx_ctl cluster join emqx@172.25.0.5

# 查询集群状态
./bin/emqx_ctl cluster status

# 退出集群
./bin/emqx_ctl cluster leave

# 或者在另一个实例中，
# ./bin/emqx_ctl cluster force-leave emqx@172.25.0.4
