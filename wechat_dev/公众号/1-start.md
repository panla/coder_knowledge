# 初识

[toc]

## 账号类型

- 公众号
  - 订阅号
  - 服务号
  - 小程序
- 企业微信(企业号)

[服务号文档](https://developers.weixin.qq.com/doc/offiaccount/Getting_Started/Overview.html)

## 基本概述

为了识别用户，每个用户针对每个公众号会产生一个安全的OpenID

微信公众平台开发 微信开放平台

公众平台以 `access_token` 为接口调用凭据，来调用接口，所有接口的调用需要先获取 `access_token`
`access_token` 在2小时内有效，过期需要重新获取，但 1 天内获取次数有限，开发者需自行存储

公众号主要通过`公众号消息会话`和`公众号内网页`来为用户提供服务的

### 服务方式，公众号消息会话

- 群发消息
- 被动回复消息
- 客户消息
- 模板消息

### 公众号内网页

- 网页授权获取用户基本信息
- 微信JS-SDK：

## 获取基本 `access_token`

```text
GET https://api.weixin.qq.com/cgi-bin/token

?grant_type=client_credential&appid={APPID}&secret={APPSECRET}

传入
    APPID
    APPSECRET

返回
    access_token
    expires_in: 7200
```
