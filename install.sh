#########################################################################
#########################################################################
######## Change lines before executing script:

### trac.laas.fr username
USERNAME_SYS=orthez
USERNAME_LAAS=aorthey

### install dir for openrobots/robotpkg etc
INSTALL_DIR=/opt

### installing the git libraries
LIB_DIR=/home/${USERNAME_SYS}/lib

### where to download git archives
GIT_DIR=/home/${USERNAME_SYS}/git
#########################################################################
#########################################################################
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:${LIB_DIR}/lib/pkgconfig:${INSTALL_DIR}/openrobots/lib/pkgconfig

### install all packages from repositories

## OMPL prerequisites
sudo apt-get install libboost-all-dev cmake curl python-dev python-qt4-dev \
python-qt4-gl python-opengl freeglut3-dev
sudo apt-get install doxygen graphviz libode-dev

##### reload robotpkg.conf
rm -rf ${INSTALL_DIR}/openrobots/etc/robotpkg.conf
echo "ACCEPTABLE_LICENSES+=pqp-license" >> ${INSTALL_DIR}/openrobots/etc/robotpkg.conf
echo "ACCEPTABLE_LICENSES+=cnrs-hpp-closed-source" >> ${INSTALL_DIR}/openrobots/etc/robotpkg.conf

###### Load all packages into GIT folder
cd ${GIT_DIR}
git clone ssh://${USERNAME_LAAS}@trac.laas.fr/git/jrl/rblink/fastReplanningData.git
git clone ssh://${USERNAME_LAAS}@trac.laas.fr/git/jrl/rblink/analyticalPG
git clone ssh://${USERNAME_LAAS}@trac.laas.fr/git/jrl/algo/multiRRT.git
git clone git://github.com/flexible-collision-library/fcl.git
git clone git://git.openrobots.org/robots/robotpkg
git clone git@github.com:danfis/libccd.git

echo "Successfully downloaded all packages"
#########################################################################
## robot-pkg
#########################################################################
cd ${GIT_DIR}
cd robotpkg/bootstrap
./bootstrap --prefix=${GIT_DIR}/openrobots

#########################################################################
## analyticalPG
#########################################################################
cd ${GIT_DIR}
cd analyticalPG
cd build
cmake .. -DCMAKE_INSTALL_PREFIX:PATH=${LIB_DIR}
make -j5
make all install

#########################################################################
## multiRRT
#########################################################################
cd ${GIT_DIR}
cd multiRRT
cmake .. -DCMAKE_INSTALL_PREFIX:PATH=${LIB_DIR}
make all install

#########################################################################
##LIBCCD (required for FCL)
#########################################################################
cd ${GIT_DIR}
cd libccd
./bootstrap
./configure --prefix=${LIB_DIR}
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX:PATH=${LIB_DIR}
make all install

#########################################################################
## FCL
#########################################################################
cd ${GIT_DIR}
cd fcl
mkdir build
cmake .. -DCMAKE_INSTALL_PREFIX:PATH=${LIB_DIR}
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
cd ${GIT_DIR}/robotpkg/math/hrp2-dynamics
make update
cd ${GIT_DIR}/robotpkg/graphics/pqp
make update
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${INSTALL_DIR}/openrobots:${LIB_DIR}/lib

