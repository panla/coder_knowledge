# use

## 系统完整性保护

```bash
command + R

csrutil disable

csrutil enable
```

## 虚拟内存

```bash
# 关闭
sudo launchctl unload -w/System/Library/LaunchDaemons/com.apple.dynamic_pager.plist

# 查看虚拟内存使用情况
sysctl vm.swapusage
```
