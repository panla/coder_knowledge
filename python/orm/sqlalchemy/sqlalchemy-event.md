# 事件监听

## 一

```python
from sqlalchemy import event

def setPassword(target, value, oldvalue, initiator):
    if value == oldvalue:#如果新设置的值与原有的值相等，那么说明用户并没有修改密码，返回原先的值
        return oldvalue
    #如果新值与旧值不同，说明密码发生改变，进行加密，加密方法可以根据自己需求改变
    return hashlib.md5("%s%s" % (password_prefix, value)).hexdigest()


#设置事件监听，event.listen(表单或表单字段, 触发事件, 回调函数, 是否改变插入值)
event.listen(User.password, "set", setPassword, retval=True)

```
