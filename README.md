# Build

### 1. [install opencv](https://docs.opencv.org/3.1.0/d7/d9f/tutorial_linux_install.html)

### 2. install dependencies:
```
sudo apt-get install libboost-all-dev libpcap-dev libyaml-cpp-dev
```
### 3. install pcl:
```
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all
```

### 4. clone and build:
```
git clone http://code.hesaitech.cn:10080/yy/pandar-ros-driver.git
cd pandar-ros-driver
cmake ..
make -j $(nproc)
```