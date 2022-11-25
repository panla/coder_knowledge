# limits

## shell

```bash
ulimit -n 100000
```

## user

`/etc/security/limits.conf`

```conf
* soft nofile 655350
* hard nofile 655350

* soft nproc 1310720
* hard nproc 1310720
```

## system

`/proc/sys/fs/file-max`

(2 ** 63) - 1
