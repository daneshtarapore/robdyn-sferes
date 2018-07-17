#!/bin/bash

set -x # verbose
set -e # stop on errors

INSTALL="$(realpath install)"
echo "Install directory: ${INSTALL}"
NO_OSG=yes
OSG=no

echo "INSTALL: $INSTALL"
echo "NO_OSG: $NO_OSG"
echo "OSG: $OSG"

rm -rf install
mkdir install

#### robdyn
# latest version of robdyn from github
cd robdyn
./waf configure --disable_osg=no --prefix=$INSTALL
./waf install
./waf install # yes, we do it again
cd ..

#### sferes
pwd
cd sferes2/exp
#ln -s ../../exp/hexa_supg_hyperneat .
cd ..
echo "robdyn" > modules.conf


CXXFLAGS='-fpermissive' ./waf configure --no-mpi --robdyn-osg --includes=/usr/local/include --libs==/usr/lib/x86_64-linux-gnu --robdyn $INSTALL --eigen3 /usr/include/eigen3 --boost-libs=/usr/lib/x86_64-linux-gnu --cpp11 yes
./waf -j4
./waf --exp hexa_supg_hyperneat -j4
