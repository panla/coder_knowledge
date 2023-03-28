# start

[TOC]

## minikube start --help

```bash
Starts a local Kubernetes cluster

Options:
    --addons=[]:
        启用插件。执行 `minikube addons list` 查看可用插件名称列表

    --apiserver-ips=[]:
        一组在为 kubernetes 生成的证书中使用的 apiserver IP 地址。如果您希望将此 apiserver
        设置为可从机器外部访问，则可以使用这组 apiserver IP 地址

    --apiserver-name='minikubeCA':
        The authoritative apiserver hostname for apiserver certificates and connectivity. This can be used if you want
        to make the apiserver available from outside the machine

    --apiserver-names=[]:
        一组在为 kubernetes 生成的证书中使用的 apiserver 名称。如果您希望将此 apiserver
        设置为可从机器外部访问，则可以使用这组 apiserver 名称

    --apiserver-port=8443:
        apiserver 侦听端口

    --auto-update-drivers=true:
        如果设置了，将自动更新驱动到最新版本。默认为 true。

    --base-image='gcr.io/k8s-minikube/kicbase:v0.0.37@sha256:8bf7a0e8a062bc5e2b71d28b35bfa9cc862d9220e234e86176b3785f685d8b15':
        The base image to use for docker/podman drivers. Intended for local development.

    --binary-mirror='':
        Location to fetch kubectl, kubelet, & kubeadm binaries from.

    --cache-images=true:
        If true, cache docker images for the current bootstrapper and load them into the machine. Always false with
        --driver=none.

    --cert-expiration=26280h0m0s:
        Duration until minikube certificate expiration, defaults to three years (26280h).

    --cni='':
        CNI plug-in to use. Valid options: auto, bridge, calico, cilium, flannel, kindnet, or path to a CNI manifest
        (default: auto)

    --container-runtime='':
        The container runtime to be used. Valid options: docker, cri-o, containerd (default: auto)

    --cpus='2':
        Number of CPUs allocated to Kubernetes. Use "max" to use the maximum number of CPUs.

    --cri-socket='':
        The cri socket path to be used.

    --delete-on-failure=false:
        If set, delete the current cluster if start fails and try again. Defaults to false.

    --disable-driver-mounts=false:
        停用由管理程序提供的文件系统装载

    --disable-metrics=false:
        If set, disables metrics reporting (CPU and memory usage), this can improve CPU usage. Defaults to false.

    --disable-optimizations=false:
        If set, disables optimizations that are set for local Kubernetes. Including decreasing CoreDNS replicas from 2
        to 1. Defaults to false.

    --disk-size='20000mb':
        分配给 minikube 虚拟机的磁盘大小（格式：<数字>[<单位>]，其中单位 = b、k、m 或
        g）。

    --dns-domain='cluster.local':
        The cluster dns domain name used in the Kubernetes cluster

    --dns-proxy=false:
        为 NAT DNS 请求启用代理（仅限 virtualbox 驱动程序）

    --docker-env=[]:
        传递给 Docker 守护进程的环境变量。（格式：键值对）

    --docker-opt=[]:
        指定要传递给 Docker 守护进程的任意标志。（格式：key=value）

    --download-only=false:
        如果为 true，仅会下载和缓存文件以备后用 - 不会安装或启动任何项。

    --driver='':
        Driver is one of: virtualbox, vmwarefusion, kvm2, qemu2, qemu, vmware, none, docker, podman, ssh (defaults to
        auto-detect)

    --dry-run=false:
        dry-run mode. Validates configuration, but does not mutate system state

    --embed-certs=false:
        if true, will embed the certs in kubeconfig.

    --enable-default-cni=false:
        DEPRECATED: Replaced by --cni=bridge

    --extra-config=:
        A set of key=value pairs that describe configuration that may be passed to different components.                The key
        should be '.' separated, and the first part before the dot is the component to apply the configuration to.
        Valid components are: kubelet, kubeadm, apiserver, controller-manager, etcd, proxy, scheduler           Valid kubeadm
        parameters: ignore-preflight-errors, dry-run, kubeconfig, kubeconfig-dir, node-name, cri-socket,
        experimental-upload-certs, certificate-key, rootfs, skip-phases, pod-network-cidr

    --extra-disks=0:
        Number of extra disks created and attached to the minikube VM (currently only implemented for hyperkit and
        kvm2 drivers)

    --feature-gates='':
        一组用于描述 alpha 版功能/实验性功能的功能限制的键值对。

    --force=false:
        强制 minikube 执行可能有风险的操作

    --force-systemd=false:
        If set, force the container runtime to use systemd as cgroup manager. Defaults to false.

    --host-dns-resolver=true:
        为 NAT DNS 请求启用主机解析器（仅限 virtualbox 驱动程序）

    --host-only-cidr='192.168.59.1/24':
        需要用于 minikube 虚拟机的 CIDR（仅限 virtualbox 驱动程序）

    --host-only-nic-type='virtio':
        网卡类型仅用于主机网络。Am79C970A, Am79C973, 82540EM, 82543GC, 82545EM 之一，或 virtio(仅限
        VirtualBox 驱动程序)

    --hyperkit-vpnkit-sock='':
        用于网络连接的 VPNKit 套接字的位置。如果为空，则停用 Hyperkit
        VPNKitSock；如果为“auto”，则将 Docker 用于 Mac VPNKit 连接；否则使用指定的
        VSock（仅限 hyperkit 驱动程序）

    --hyperkit-vsock-ports=[]:
        应在主机上公开为套接字的访客 VSock 端口列表（仅限 hyperkit 驱动程序）

    --hyperv-external-adapter='':
        External Adapter on which external switch will be created if no external switch is found. (hyperv driver only)

    --hyperv-use-external-switch=false:
        Whether to use external switch over Default Switch if virtual switch not explicitly specified. (hyperv driver
        only)

    --hyperv-virtual-switch='':
        hyperv 虚拟交换机名称。默认为找到的第一个 hyperv 虚拟交换机。（仅限 hyperv
        驱动程序）

    --image-mirror-country='':
        需要使用的镜像镜像的国家/地区代码。留空以使用全球代码。对于中国大陆用户，请将其设置为
        cn。

    --image-repository='':
        用于从中拉取 docker 镜像的备选镜像存储库。如果您对 gcr.io
        的访问受到限制，则可以使用该镜像存储库。将镜像存储库设置为“auto”可让
        minikube 为您选择一个存储库。对于中国大陆用户，您可以使用本地 gcr.io 镜像，例如
        registry.cn-hangzhou.aliyuncs.com/google_containers

    --insecure-registry=[]:
        Insecure Docker registries to pass to the Docker daemon.  The default service CIDR range will automatically be
        added.

    --install-addons=true:
        If set, install addons. Defaults to true.

    --interactive=true:
        允许用户提示以获取更多信息


    --iso-url=[https://storage.googleapis.com/minikube/iso/minikube-v1.29.0-amd64.iso,https://github.com/kubernetes/minikube/releases/download/v1.29.0/minikube-v1.29.0-amd64.iso,https://kubernetes.oss-cn-hangzhou.aliyuncs.com/minikube/iso/minikube-v1.29.0-amd64.iso]:
        Locations to fetch the minikube ISO from.

    --keep-context=false:
        这将保留现有 kubectl 上下文并创建 minikube 上下文。

    --kubernetes-version='':
        The Kubernetes version that the minikube VM will use (ex: v1.2.3, 'stable' for v1.26.1, 'latest' for v1.26.1).
        Defaults to 'stable'.

    --kvm-gpu=false:
        在 minikube 中启用实验性 NVIDIA GPU 支持

    --kvm-hidden=false:
        向 minikube 中的访客隐藏管理程序签名（仅限 kvm2 驱动程序）

    --kvm-network='default':
        The KVM default network name. (kvm2 driver only)

    --kvm-numa-count=1:
        Simulate numa node count in minikube, supported numa node count range is 1-8 (kvm2 driver only)

    --kvm-qemu-uri='qemu:///system':
        KVM QEMU 连接 URI。（仅限 kvm2 驱动程序）

    --listen-address='':
        IP Address to use to expose ports (docker and podman driver only)

    --memory='':
        Amount of RAM to allocate to Kubernetes (format: <number>[<unit>], where unit = b, k, m or g). Use "max" to
        use the maximum amount of memory.

    --mount=false:
        This will start the mount daemon and automatically mount files into minikube.

    --mount-9p-version='9p2000.L':
        Specify the 9p version that the mount should use

    --mount-gid='docker':
        用于挂载默认的 group id

    --mount-ip='':
        Specify the ip that the mount should be setup on

    --mount-msize=262144:
        The number of bytes to use for 9p packet payload

    --mount-options=[]:
        其他挂载选项，例如：cache=fscache

    --mount-port=0:
        Specify the port that the mount should be setup on, where 0 means any free port.

    --mount-string='/home/user:/minikube-host':
        The argument to pass the minikube mount command on start.

    --mount-type='9p':
        Specify the mount filesystem type (supported types: 9p)

    --mount-uid='docker':
        用于挂载默认的 user id

    --namespace='default':
        The named space to activate after start

    --nat-nic-type='virtio':
        NIC Type used for nat network. One of Am79C970A, Am79C973, 82540EM, 82543GC, 82545EM, or virtio (virtualbox
        driver only)

    --native-ssh=true:
        Use native Golang SSH client (default true). Set to 'false' to use the command line 'ssh' command when
        accessing the docker machine. Useful for the machine drivers when they will not start with 'Waiting for SSH'.

    --network='':
        network to run minikube with. Now it is used by docker/podman and KVM drivers. If left empty, minikube will
        create a new network.

    --network-plugin='':
        已弃用，改用 --cni 来代替

    --nfs-share=[]:
        通过 NFS 装载与访客共享的本地文件夹（仅限 hyperkit 驱动程序）

    --nfs-shares-root='/nfsshares':
        NFS 共享的根目录位置，默认为 /nfsshares（仅限 hyperkit 驱动程序）

    --no-kubernetes=false:
        If set, minikube VM/container will start without starting or configuring Kubernetes. (only works on new
        clusters)

    --no-vtx-check=false:
        禁用在启动虚拟机之前检查硬件虚拟化的可用性（仅限 virtualbox 驱动程序）

    -n, --nodes=1:
        The number of nodes to spin up. Defaults to 1.

    -o, --output='text':
        Format to print stdout in. Options include: [text,json]

    --ports=[]:
        List of ports that should be exposed (docker and podman driver only)

    --preload=true:
        If set, download tarball of preloaded images if available to improve start time. Defaults to true.

    --qemu-firmware-path='':
        Path to the qemu firmware file. Defaults: For Linux, the default firmware location. For macOS, the brew
        installation location. For Windows, C:\Program Files\qemu\share

    --registry-mirror=[]:
        传递给 Docker 守护进程的注册表镜像

    --service-cluster-ip-range='10.96.0.0/12':
        需要用于服务集群 IP 的 CIDR。

    --socket-vmnet-client-path='':
        Path to the socket vmnet client binary (QEMU driver only)

    --socket-vmnet-path='':
        Path to socket vmnet binary (QEMU driver only)

    --ssh-ip-address='':
        IP address (ssh driver only)

    --ssh-key='':
        SSH key (ssh driver only)

    --ssh-port=22:
        SSH port (ssh driver only)

    --ssh-user='root':
        SSH user (ssh driver only)

    --static-ip='':
        Set a static IP for the minikube cluster, the IP must be: private, IPv4, and the last octet must be between 2
        and 254, for example 192.168.200.200 (Docker and Podman drivers only)

    --subnet='':
        Subnet to be used on kic cluster. If left empty, minikube will choose subnet address, beginning from
        192.168.49.0. (docker and podman driver only)

    --trace='':
        Send trace events. Options include: [gcp]

    --uuid='':
        提供虚拟机 UUID 以恢复 MAC 地址（仅限 hyperkit 驱动程序）

    --vm=false:
        Filter to use only VM Drivers

    --vm-driver='':
        DEPRECATED, use `driver` instead.

    --wait=[apiserver,system_pods]:
        comma separated list of Kubernetes components to verify and wait for after starting a cluster. defaults to
        "apiserver,system_pods", available options: "apiserver,system_pods,default_sa,apps_running,node_ready,kubelet"
        . other acceptable values are 'all' or 'none', 'true' and 'false'

    --wait-timeout=6m0s:
        max time to wait per Kubernetes or host to be healthy.

Usage:
  minikube start [flags] [options]

Use "minikube options" for a list of global command-line options (applies to all commands).
```

## example

```bash
minikube start --driver=docker --container-runtime=containerd --extra-config=kubelet.cgroup-driver=systemd --image-mirror-country='cn'

--container-runtime=The container runtime to be used.
    auto default
    docker
    cri-o
    containerd

```
