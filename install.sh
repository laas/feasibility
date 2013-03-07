export
PKG_CONFIG_PATH=$PKG_CONFIG_PATH:~/lib/lib/pkgconfig:/home/aorthey/openrobots/lib/pkgconfig
cd
mkdir git
mkdir lib

#########################################################################
## robot-pkg
#########################################################################
cd ~/git
git clone git://git.openrobots.org/robots/robotpkg
cd robotpkg/bootstrap
./bootstrap --prefix=${HOME}/openrobots

#########################################################################
## analyticalPG
#########################################################################
cd ~/git
git clone ssh://aorthey@trac.laas.fr/git/jrl/rblink/analyticalPG
cd analyticalPG
cd build
cmake .. -DCMAKE_INSTALL_PREFIX:PATH=~/lib
make -j5
make all install

#########################################################################
## multiRRT
#########################################################################
cd ~/git
git clone ssh://aorthey@trac.laas.fr/git/jrl/algo/multiRRT.git
cd multiRRT
cmake .. -DCMAKE_INSTALL_PREFIX:PATH=~/lib
make all install

#########################################################################
##LIBCCD (required for FCL)
#########################################################################
cd ~/git
git clone git@github.com:danfis/libccd.git
cd libccd
./bootstrap
./configure --prefix=/home/aorthey/lib
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX:PATH=~/lib
make all install

#########################################################################
## FCL
#########################################################################
cd ~/git
git clone git://github.com/flexible-collision-library/fcl.git
cd fcl
mkdir build
cmake .. -DCMAKE_INSTALL_PREFIX:PATH=~/lib
make -j5 all install

#########################################################################
## robot-pkg available packages
#########################################################################
cd ~/git/robotpkg/interfaces/jrl-mal
make update
cd ~/git/robotpkg/motion/dynamic-graph
make update
cd ~/git/robotpkg/motion/dynamic-graph-corba
make update
cd ~/git/robotpkg/net/evart-client
make update
echo "ACCEPTABLE_LICENSES+=cnrs-hpp-closed-source" >> /home/aorthey/openrobots/etc/robotpkg.conf
cd ~/git/robotpkg/math/hrp2-dynamics
make update
echo "ACCEPTABLE_LICENSES+=pqp-license" >> /home/aorthey/openrobots/etc/robotpkg.conf
cd ~/git/robotpkg/graphics/pqp
make update
export
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/aorthey/openrobots:/home/aorthey/lib/lib

cd ~/git
git clone ssh://aorthey@trac.laas.fr/git/jrl/rblink/fastReplanningData.git
