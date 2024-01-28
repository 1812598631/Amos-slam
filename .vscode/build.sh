#工作目录
WORKSPACE_DIR=/home/run/slam/ORB-SLAM-yolact
BUILD_DIR=${WORKSPACE_DIR}/build
mkdir $BUILD_DIR
cd $BUILD_DIR
# cmake -DCMAKE_BUILD_TYPE=Debug  ..
cmake -DCMAKE_BUILD_TYPE=Release   ..

make orb_yolact -j16