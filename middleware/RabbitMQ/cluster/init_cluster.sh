#reset first node
echo "Reset first rabbitmq node.================================================================"
docker exec dev_study_rabbitmq_11 /bin/bash -c 'rabbitmqctl stop_app'
docker exec dev_study_rabbitmq_11 /bin/bash -c 'rabbitmqctl reset'
docker exec dev_study_rabbitmq_11 /bin/bash -c 'rabbitmqctl start_app'

#build cluster
echo "Starting to build rabbitmq cluster with two ram nodes.===================================="
docker exec dev_study_rabbitmq_12 /bin/bash -c 'rabbitmqctl stop_app'
docker exec dev_study_rabbitmq_12 /bin/bash -c 'rabbitmqctl reset'
docker exec dev_study_rabbitmq_12 /bin/bash -c 'rabbitmqctl join_cluster --ram rabbit@myNode-11'
docker exec dev_study_rabbitmq_12 /bin/bash -c 'rabbitmqctl start_app'

docker exec dev_study_rabbitmq_13 /bin/bash -c 'rabbitmqctl stop_app'
docker exec dev_study_rabbitmq_13 /bin/bash -c 'rabbitmqctl reset'
docker exec dev_study_rabbitmq_13 /bin/bash -c 'rabbitmqctl join_cluster --ram rabbit@myNode-11'
docker exec dev_study_rabbitmq_13 /bin/bash -c 'rabbitmqctl start_app'

#check cluster status
echo "Check cluster status:====================================================================="
docker exec dev_study_rabbitmq_11 /bin/bash -c 'rabbitmqctl cluster_status'
docker exec dev_study_rabbitmq_12 /bin/bash -c 'rabbitmqctl cluster_status'
docker exec dev_study_rabbitmq_13 /bin/bash -c 'rabbitmqctl cluster_status'
