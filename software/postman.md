# 安装 Postman

## 下载

[下载地址](https://www.postman.com/downloads/)

## desktop

```text
[Desktop Entry]
Type=Application
Categories=Development
Encoding=UTF-8
Name=Postman
Exec=/home/opt/Postman/app/Postman
Icon=/home/opt/Postman/app/resources/app/assets/icon.png
Terminal=false
```

## error

```bash
# error while loading shared libraries: libXss.so.1
dnf install libXScrnSaver
```

## 变量

```javascript
//把json字符串转化为对象
var data=JSON.parse(responseBody);

//获取data对象的 token 值。
var token=data.token;

//设置成全局变量
pm.globals.set("token_local", token);


var csrf_token = postman.getResponseCookie("csrftoken").value;
// postman.clearGlobalVariable("local_csrf");
postman.setGlobalVariable("local_csrf", csrf_token);
```
