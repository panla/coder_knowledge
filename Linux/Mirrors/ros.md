# ros

## ros melodic ubuntu 18.04

```bash
# add repo
touch /etc/apt/sources.list.d/ros-latest.list

echo deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ bionic main >> /etc/apt/sources.list.d/ros-latest.list

# add key
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# update
sudo apt update

# install
sudo apt install ros-melodic-desktop-full
```

## ros noetic ubuntu 20.04

```bash
# add repo
touch /etc/apt/sources.list.d/ros-latest.list

echo deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ focal main >> /etc/apt/sources.list.d/ros-latest.list

# add key
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# update
sudo apt update

# install
sudo apt install ros-noetic-desktop-full
```

## ros2 humble ubuntu 22.04

```bash
# add key
sudo apt install curl gnupg2
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

# add repo
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# update
sudo apt update

# install
sudo apt install ros-humble-desktop
```
