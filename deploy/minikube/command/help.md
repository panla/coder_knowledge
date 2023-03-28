# command

[TOC]

## minikube --help

```bash
minikube --help

minikube provisions and manages local Kubernetes clusters optimized for development workflows.

Basic Commands:
  start            Starts a local Kubernetes cluster
  status           Gets the status of a local Kubernetes cluster
  stop             Stops a running local Kubernetes cluster
  delete           Deletes a local Kubernetes cluster
  dashboard        访问在 minikube 集群中运行的 kubernetes dashboard
  pause            pause Kubernetes
  unpause          恢复 Kubernetes

Images Commands:
  docker-env       Provides instructions to point your terminal's docker-cli to the Docker Engine inside minikube.
(Useful for building docker images directly inside minikube)
  podman-env       配置环境以使用 minikube's Podman service
  cache            Manage cache for images
  image            Manage images

Configuration and Management Commands:
  addons           Enable or disable a minikube addon
  config           Modify persistent configuration values
  profile          Get or list the current profiles (clusters)
  update-context   Update kubeconfig in case of an IP or port change

Networking and Connectivity Commands:
  service          Returns a URL to connect to a service
  tunnel           连接到 LoadBalancer 服务

Advanced Commands:
  mount            将指定的目录挂载到 minikube
  ssh              Log into the minikube environment (for debugging)
  kubectl          Run a kubectl binary matching the cluster version
  node             Add, remove, or list additional nodes
  cp               Copy the specified file into minikube

Troubleshooting Commands:
  ssh-key          Retrieve the ssh identity key path of the specified node
  ssh-host         Retrieve the ssh host key of the specified node
  ip               Retrieves the IP address of the specified node
  logs             Returns logs to debug a local Kubernetes cluster
  update-check     打印当前和最新版本版本
  version          打印 minikube 版本
  options          显示全局命令行选项列表 (应用于所有命令)。

Other Commands:
  completion       Generate command completion for a shell
  license          Outputs the licenses of dependencies to a directory

Use "minikube <command> --help" for more information about a given command.
```
