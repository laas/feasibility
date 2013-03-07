#########################################################################
#########################################################################
######## Change lines before executing script:

### trac.laas.fr username
USERNAME=aorthey

### install dir for openrobots/robotpkg etc
INSTALL_DIR=/home/aorthey

### installing the git libraries
LIB_DIR=/home/aorthey/lib

### where to download git archives
GIT_DIR=/home/aorthey/git
#########################################################################
#########################################################################


export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:${LIB_DIR}/lib/pkgconfig:${INSTALL_DIR}/openrobots/lib/pkgconfig

#########################################################################
## robot-pkg
#########################################################################
cd ${GIT_DIR}
git clone git://git.openrobots.org/robots/robotpkg
cd robotpkg/bootstrap
./bootstrap --prefix=${HOME}/openrobots

#########################################################################
## analyticalPG
#########################################################################
cd ${GIT_DIR}
git clone ssh://${USERNAME}@trac.laas.fr/git/jrl/rblink/analyticalPG
cd analyticalPG
cd build
cmake .. -DCMAKE_INSTALL_PREFIX:PATH=~/lib
make -j5
make all install

#########################################################################
## multiRRT
#########################################################################
cd ${GIT_DIR}
git clone ssh://${USERNAME}@trac.laas.fr/git/jrl/algo/multiRRT.git
cd multiRRT
cmake .. -DCMAKE_INSTALL_PREFIX:PATH=~/lib
make all install

#########################################################################
##LIBCCD (required for FCL)
#########################################################################
cd ${GIT_DIR}
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
cd ${GIT_DIR}
git clone git://github.com/flexible-collision-library/fcl.git
cd fcl
mkdir build
cmake .. -DCMAKE_INSTALL_PREFIX:PATH=~/lib
make -j5 all install

#########################################################################
## robot-pkg available packages
#########################################################################
cd ${GIT_DIR}/robotpkg/interfaces/jrl-mal
make update
cd ${GIT_DIR}/robotpkg/motion/dynamic-graph
make update
cd ${GIT_DIR}/robotpkg/motion/dynamic-graph-corba
make update
cd ${GIT_DIR}/robotpkg/net/evart-client
make update
echo "ACCEPTABLE_LICENSES+=cnrs-hpp-closed-source" >> ${INSTALL_DIR}/openrobots/etc/robotpkg.conf
cd ${GIT_DIR}/robotpkg/math/hrp2-dynamics
make update
echo "ACCEPTABLE_LICENSES+=pqp-license" >> ${INSTALL_DIR}/openrobots/etc/robotpkg.conf
cd ${GIT_DIR}/robotpkg/graphics/pqp
make update
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${INSTALL_DIR}/openrobots:${LIB_DIR}/lib

cd ~/git
git clone ssh://aorthey@trac.laas.fr/git/jrl/rblink/fastReplanningData.git
