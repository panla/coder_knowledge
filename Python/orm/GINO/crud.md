# CRUD

[toc]

## 连接

```python
db = Gino()

async with db.with_bind('postgresql://localhost/gino'):
```

## 增

```python
user = await User.create(nickname='fantix')
user = await User(nickname='pandora').create()
```

## 查

```python
user = await User.get(1)
users = await db.all(User.query)
users = await User.query.gino.all()
users = await User.query.where(User.id < 10).gino.all()
user = await User.query.where(User.nickname == 'fantix').gino.first()

# id=1 的 user.nickname
name = await User.select('nickname').where(User.id == 1).gino.scalar()
# 查询数量
population = await db.func.count(User.id).gino.scalar()
## 连接查询
results = await User.join(Book).select().gino.all()
```

## 改

```python
await user.update(nickname='daisy').apply()
await User.update.values(nickname='pandora' += ' is good').where(User.id < 10).gino.status()
```

## 删

```python
await user.delete()
await User.delete.where(User.id > 10).gino.status()
```

## 指定 bind

```python
engine = await create_engine("...")
# ...
user = await User.get(request.user_id, bind=engine)
```
