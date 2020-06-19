#### Pre-requiste

### Dependecies
```
sudo apt-get install cmake liblz4-dev
```

### flann

Version 1.9.1 : https://github.com/mariusmuja/flann

```
git clone https://github.com/mariusmuja/flann.git
cd flann
git checkout -b 1.9.1 tags/1.9.1
mkdir build
cd build
cmake ..
make
```

### protobuf

Currently imported from OR-Tools as it is already embed with others Mapotempo projects

Download and extract last asset: https://github.com/google/or-tools/releases/download/v7.5/or-tools_debian-10_v7.5.7466.tar.gz

#### BalancedVRPClustering

Ensure path of OR_TOOLS_TOP and FLANN_MASTER_TOP are correct in the Makefile

compile
```
make  clustering
```

clean
```
make mrproper
```

execute
```
./clustering instance-13.bin
```
