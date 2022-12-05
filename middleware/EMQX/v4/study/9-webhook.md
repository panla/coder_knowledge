# WebHook

<https://www.emqx.io/docs/zh/v4.4/advanced/webhook.html>

## conf

```conf
## The web services URL for Hook request
##
## Value: String
web.hook.url = http://127.0.0.1:8080

## Encode message payload field
##
## Value: base64 | base62
## web.hook.encode_payload = base64

##--------------------------------------------------------------------
## Hook Rules

## These configuration items represent a list of events should be forwarded
##
## Format:
##   web.hook.rule.<HookName>.<No> = <Spec>
web.hook.rule.client.connect.1       = {"action": "on_client_connect"}
web.hook.rule.client.connack.1       = {"action": "on_client_connack"}
web.hook.rule.client.connected.1     = {"action": "on_client_connected"}
web.hook.rule.client.disconnected.1  = {"action": "on_client_disconnected"}
web.hook.rule.client.subscribe.1     = {"action": "on_client_subscribe"}
web.hook.rule.client.unsubscribe.1   = {"action": "on_client_unsubscribe"}
web.hook.rule.session.subscribed.1   = {"action": "on_session_subscribed"}
web.hook.rule.session.unsubscribed.1 = {"action": "on_session_unsubscribed"}
web.hook.rule.session.terminated.1   = {"action": "on_session_terminated"}
web.hook.rule.message.publish.1      = {"action": "on_message_publish"}
web.hook.rule.message.delivered.1    = {"action": "on_message_delivered"}
web.hook.rule.message.acked.1        = {"action": "on_message_acked"}
```
