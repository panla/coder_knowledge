# GitLab CI/CD

## 概述

## .giblab-ci.yml

`.gitlab-ci.yml` 被用来管理项目的 runner 任务，存放于项目根目录

- [语法参考-1](https://gitlab.boonray.com/help/ci/yaml/README)

### 关键字

一级

```text
image 和 services
    允许使用一个自定义的 Docker 镜像和一系列服务，并且可以用于整个 job 周期

before_script
    定义所有 job 运行之前的命令
after_script

stages
    决定对应 job 的执行顺序
    定义被 job 调用的 stages, 允许有多级的 pipelines
    相同 stage 的 job 可以平行执行
    下一个 stage 的 job 会在前一个 stage 的 job 成功后执行
    未定义 stages 时，默认为 build
    一个 job 未指定 stage 时，会分配到 test 部分
stages:
  - build
  - test
  - deploy
    1 所有相同 stage 的 jobs 是并行执行
    2 所有相同 stage 的 jobs 执行成功后，下一个 stage 才会执行
    3 所有 build,test,deploy 的 jobs 执行完，此 commit 标记为 success
    4 有一个前置的 job 失败时，此 commit 标记为 failed 并且下一个 stages 的 job 不会执行

variables
    允许在 .gitlab-ci.yml 中添加变量
```

二级

```text
job
    stage
        该任务所属 stage

    script
        脚本命令

    only
        定义一列git分支，并为其创建job

    except
        定义分支，不创建 job

    tags
        定义 tags 用来指定用哪个 Runner
        注册 runner 时的 tag

    allow_failure: true
        允许失败

    when: on_success, on_failure, always, manual
        何时执行 Job

    before_script
        定义 job 运行之前的命令
    after_script

```

## GitLab-Runner

GitLab Runner是一个开源项目，用于运行作业（jobs）并将结果发送回GitLab。
它与GitLab CI结合使用，GitLab CI是GitLab用于协调jobs的开源持续集成服务。

### install and setup

```bash
docker run -d --name gitlab-runner --restart always \ 
-v /srv/gitlab-runner/config:/etc/gitlab-runner \ 
-v /var/run/docker.sock:/var/run/docker.sock \ 
gitlab/gitlab-runner:latest
```

### register

```bash
docker exec -it gitlab-runner bash

gitlab-runner register

Running in system-mode.

Please enter the gitlab-ci coordinator URL (e.g. https://gitlab.com/):
>> http://xx.com

Please enter the gitlab-ci token for this runner:
>> qwertyuiopasdfghjkl

Please enter the gitlab-ci description for this runner:
>> python_runner_demo

Please enter the gitlab-ci tags for this runner (comma separated):
>> python_demo

Please enter the executor: docker, shell, ssh, kubernetes, docker-ssh, parallels, virtualbox, docker+machine, docker-ssh+machine:
>> docker

default image
>> alpine:latest
```

## 优缺点

总的来说，gitlab-ci基本上可以完成完整的构建及发布，但也会存在一些缺点:
发布部分，需要将程序部署到哪个服务器固化到.gitlab-ci文件中。
另外，如果runner上直接进行部署，那么runner所在的机器则需要直接或间接的访问所有的发布的机器，这里存在一定安全问题．
