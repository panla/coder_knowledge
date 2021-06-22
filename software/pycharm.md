# Linux 安装

## 下载解压

## 权限

```bash
ch /opt/PyCharm/bin
chmod 777 ./*
```

## jdk

```text
JAVA_BIN="/usr/bin/java"
```

## idea.properties

```text
idea.config.path=/home/opt/PyCharm-202032/PyCharmCE/config
idea.system.path=/home/opt/PyCharm-202032/PyCharmCE/system
idea.plugins.path=/home/opt/PyCharm-202032/PyCharmCE/config/plugins
idea.log.path=/home/opt/PyCharm-202032/PyCharmCE/system/log
```

## pycharm64.vmoptions

```text
-Xms200m
-Xmx350m
-XX:ReservedCodeCacheSize=300m
-XX:+UseConcMarkSweepGC
-XX:SoftRefLRUPolicyMSPerMB=100
```

## 桌面图标

```text
[Desktop Entry]
Type=Application
Name=PyCharm 202112
GenericName=Python IDE
Comment=Python IDE
Exec=sh /home/opt/PyCharm-202112/bin/pycharm.sh
Icon=/home/opt/PyCharm-202112/bin/pycharm.png
Categories=IDE;
Actions=new-empty-window;
Terminal=PyCharm

[Desktop Action new-empty-window]
Name=New Empty Window
Exec=sh /home/opt/PyCharm-202112/bin/pycharm.sh
Icon=/home/opt/PyCharm-202112/bin/pycharm.png
Terminal=PyCharm
```

## 插件

```text
.ignore
ideolog
Ini
Rainbow Brackets
HighlightBracketPair

Settings Repository

Makrdown
properties
Shell Script
YAML

ChangeReminder
Git
GitHub
Mercurial
Subversion

Copyright
EditorConfig
Grazie
IDE Features Trainer
IntelliLang
Machine Learning Code Completion
RestructuredText
Task Management
Termial
TextMate Bundles
```
