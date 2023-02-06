# apt proxy

## 1

```bash
sudo apt -o Acquire::http::proxy="http://127.0.0.1:8000/" update
```

## 2

```bash
vim /etc/apt/apt.conf.d/1.conf

# add
Acquire::http::proxy "http://user:pwd@127.0.0.1:8000"
```
