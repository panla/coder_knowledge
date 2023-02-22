# Image

升级内核

不要装 hwe 内核，占用很多硬盘空间，很有可能安装不上

## command

```bash
sudo dpkg -l | egrep "linux-header|linux-image"


sudo apt search linux-image-5
sudo apt search linux-image-5.18
sudo apt search linux-image-5.18.17

sudo apt search linux-headers-5.*

sudo apt install linux-image-5.17.3-amd64-desktop
sudo apt install linux-headers-5.17.3-amd64-desktop

sudo grub-mkconfig

sudo apt purge linux-headers-5.xx.xx-xxxx
```
