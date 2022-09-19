# 端口

## 一般

- 1883: MQTT/TCP - External TCP Listener for MQTT Protocol
- 8883: MQTT/SSL - External SSL Listener for MQTT Protocol
- 8083: External WebSocket listener for MQTT protocol
- 8084: External WebSocket/SSL listener for MQTT Protocol
- 18083: The port that the Dashboard HTTP listener will bind.
- 18084: The port that the Dashboard HTTPS listener will bind.
- 11883: The IP address and port that the internal MQTT/TCP protocol listener

## 集群

- 4369 4370: Cluster Multicast Ports
- 5369: 集群端口
  - TCP port number for RPC server to listen on.
  - Only takes effect when `rpc.port_discovery` = `manual`.
  - All nodes in the cluster should agree to this same config.
- 5370: RPC port discovery
  - The strategy for discovering the RPC listening port of other nodes.
