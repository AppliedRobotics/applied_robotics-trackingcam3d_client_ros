
#1) COPY this SCRIPT and clone 5 tcam libs source into the SAME folder:
# libtconfig
# libtlive
#2) Run script, wait, be ready to enter sudo password

parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd "$parent_path"

cd applied_robotics-libtconfig-7df806401f29/build/
sudo rm -r *
cmake -D CMAKE_BUILD_TYPE=RELEASE ..
make -j2
sudo make install
cd ../..

cd applied_robotics-libtlive-1e2c96f504a3/build/
sudo rm -r *
cmake -D CMAKE_BUILD_TYPE=RELEASE ..
make -j2
sudo make install
cd ../..

 
