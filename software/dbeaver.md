# DBeaver

## 下载

[github](https://github.com/dbeaver/dbeaver/releases)
[代下-1](https://www.offcloud.com/)
[代下-2](https://shrill-pond-3e81.hunsh.workers.dev/)

## 安装-配置

窗口->首选项->连接->驱动->maven->添加

```text
name
https://maven.aliyun.com/repository/public
```

## build

```bash
git clone https://github.com/dbeaver/dbeaver.git dbeaver
git clone git@gitee.com:mirrors/dbeaver.git dbeaver
cd dbeaver
# 需要已经配置 java maven
mvn package
```

## desktop

```text
[Desktop Entry]
Version=1.0
Type=Application
Terminal=false
Name=DBeaver Community
GenericName=UniversaL Database Manager
Comment=Universal Database Manager and SQL Client.
Path=/opt/dbeaver/
Exec=/opt/dbeaver/dbeaver
Icon=/opt/dbeaver/dbeaver.png
Categories=IDE;Development
WM_CLASS=DBeaver
StartupWMClass=DBeaver
StartupNotify=true
Keywords=Database;SQL;IDE;JDBC;ODBC;MySQL;PostgreSQL;Oracle;DB2;MariaDB
MimeType=application/sql
```
