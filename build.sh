echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=/home/schoerghuberm/work/msptl/platforms/ci/install/opencv-3.4.2/share/OpenCV
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DEIGEN3_INCLUDE_DIR=/home/schoerghuberm/work/msptl/cmake-build-relwithdebinfo/Eigen/include/eigen3
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=/home/schoerghuberm/work/msptl/platforms/ci/install/opencv-3.4.2/share/OpenCV
-Dopengv_DIR=/home/schoerghuberm/work/msptl/platforms/ci/install/opengv/CMake
-Dg2o_DIR=/home/schoerghuberm/work/msptl/platforms/ci/install/g2o/CMake
-DPangolin_DIR=/home/schoerghuberm/work/msptl/platforms/ci/install/pangolin/lib/cmake/Pangolin -DEIGEN3_INCLUDE_DIR=/home/schoerghuberm/work/msptl/cmake-build-relwithdebinfo/Eigen/include/eigen3
make -j
