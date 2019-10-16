
echo "Configuring and building Thirdparty/DBoW2 ..."

root_dir=`pwd`
thirdparty_dir=`pwd`/Thirdparty

## to build on your development PC, install locally and use custom system opencv, just uncomment and manually add paths to your cmake for orb
#LOCAL_INSTALL_Eigen="-DCMAKE_INSTALL_PREFIX=${thirdparty_dir}/eigen"
#LOCAL_INSTALL_Pangolin="-DCMAKE_INSTALL_PREFIX=${thirdparty_dir}/pangolin"
#LOCAL_INSTALL_Eigen_cmake="-DEigen3_DIR=${thirdparty_dir}/eigen/share/eigen3/cmake"
#LOCAL_INSTALL_Eigen_include="-DEIGEN_INCLUDE_DIR=${thirdparty_dir}/eigen/include/eigen3"
#LOCAL_OPENCV="-DOpenCV_DIR=/opt/ait/thirdparty/opencv-3.4.2/share/OpenCV"

cd ${thirdparty_dir}

wget -q http://bitbucket.org/eigen/eigen/get/3.3.7.tar.bz2
tar xf 3.3.7.tar.bz2
rm -rf 3.3.7.tar.bz2
mv eigen-eigen-* eigen-eigen-3.3.7
cd eigen-eigen-3.3.7
mkdir -p build && cd build
cmake ..  -DCMAKE_BUILD_TYPE=Release ${LOCAL_INSTALL_Eigen}
make -j 4
make install

cd ${thirdparty_dir}
git clone https://github.com/stevenlovegrove/Pangolin.git Pangolin_src
mkdir -p Pangolin_src/build && cd Pangolin_src/build
cmake .. -DCMAKE_BUILD_TYPE=Release ${LOCAL_INSTALL_Eigen_include} ${LOCAL_INSTALL_Pangolin} -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF  -DBUILD_PANGOLIN_LIBREALSENSE2=OFF  -DBUILD_PANGOLIN_LIBREALSENSE=OFF
make -j 4
make install

INSTALL_easylogging=${thirdparty_dir}/easyloggingpp
git clone https://github.com/zuhd-org/easyloggingpp.git -b v9.96.7  ${thirdparty_dir}/easyloggingppsrc
cd ${thirdparty_dir}/easyloggingppsrc
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=${INSTALL_easylogging}
make install -j 4


cd ${thirdparty_dir}/DBoW2
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release ${LOCAL_OPENCV}
make -j 4

cd ${thirdparty_dir}/g2o
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release ${LOCAL_INSTALL_Eigen_cmake} -DG2O_USE_OPENMP=OFF
make -j 4

cd ${root_dir}
echo "Uncompress vocabulary ..."
cd Vocabulary
tar -xf ORBvoc.txt.tar.gz

## cleanup
rm -rf ${thirdparty_dir}/Pangolin_src
rm -rf ${thirdparty_dir}/DBoW2/build
rm -rf ${thirdparty_dir}/g2o/build
rm -rf ${thirdparty_dir}/easyloggingppsrc
rm -rf ${thirdparty_dir}/eigen-eigen-3.3.7