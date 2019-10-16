echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=/opt/ait/thirdparty/opencv-3.4.2/share/OpenCV
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3
make -j


echo "Configuring and building easylogging ..."
git clone https://github.com/zuhd-org/easyloggingpp.git -b v9.96.7
cd easyloggingpp
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=install
make install


cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=/opt/ait/thirdparty/opencv-3.4.2/share/OpenCV -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3
make -j


