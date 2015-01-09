#!/bin/sh -x

#===============================================================================

SRCDIR=`pwd`
OS_BUILDDIR=$SRCDIR/iPhoneOS_build
SIMULATOR_BUILDDIR=$SRCDIR/iPhoneSimulator_build
PACKAGE_NAME=`basename $1`

#===============================================================================
echo "Installing CMake iOS toolchain ..."
if [ ! -d ios-cmake ]
    then
    git clone https://github.com/furushchev/ios-cmake.git
fi

#===============================================================================
echo "Checking for messages or services ..."

#TODO: parse the manifest to get messages dependencies
if [ -d $1/msg ] || [ -d $1/srv ];
    then
        sh $SRCDIR/messages_gen.sh -d $1 $SRCDIR/std_msgs $SRCDIR/common_msgs/geometry_msgs $SRCDIR/common_msgs/sensor_msgs $SRCDIR/common_msgs/nav_msgs
        mv $SRCDIR/$PACKAGE_NAME/*.h $1/include/$PACKAGE_NAME/
        rm -r $SRCDIR/$PACKAGE_NAME/
fi

#===============================================================================
echo "Generating cmake submodules ..."

sh $SRCDIR/cmake_gen.sh $*

#===============================================================================
echo "Generating CMakeLists.txt ..."

cat > CMakeLists.txt <<EOF
cmake_minimum_required(VERSION 2.8.0)

set (CMAKE_FRAMEWORK_PATH \${CMAKE_SYSTEM_FRAMEWORK_PATH} $SRCDIR/frameworks)

project($PACKAGE_NAME)

include($PACKAGE_NAME.cmake)

EOF

#===============================================================================
echo "Building ..."

[ -d $OS_BUILDDIR ] && rm -rf $OS_BUILDDIR
[ -d $SIMULATOR_BUILDDIR ] && rm -rf $SIMULATOR_BUILDDIR

mkdir $OS_BUILDDIR
mkdir $SIMULATOR_BUILDDIR

IOS32=`mktemp -d $OS_BUILDDIR/ios32.XXXXXX`
IOS64=`mktemp -d $OS_BUILDDIR/ios64.XXXXXX`
SIM32=`mktemp -d $OS_BUILDDIR/sim32.XXXXXX`
SIM64=`mktemp -d $OS_BUILDDIR/sim64.XXXXXX`
ARCHS_IOS_32BIT="armv7 armv7s"
ARCHS_IOS_64BIT="armv7 armv7s arm64"
ARCHS_SIM_32BIT="i386"
ARCHS_SIM_64BIT="x86_64 i386"

cd $OS_BUILDDIR

cmake -DCMAKE_TOOLCHAIN_FILE=./ios-cmake/toolchain/iOS.cmake -GXcode ..

if (! xcodebuild -sdk iphoneos -configuration Release -target ALL_BUILD ARCHS="${ARCHS_IOS_32BIT}" ONLY_ACTIVE_ARCH=NO TARGET_BUILD_DIR="${IOS32}" ) ; then
    exit 1
fi

if (! xcodebuild -sdk iphoneos -configuration Release -target ALL_BUILD ARCHS="${ARCHS_IOS_64BIT}" ONLY_ACTIVE_ARCH=NO TARGET_BUILD_DIR="${IOS64}" ) ; then
    exit 1
fi

cd $SIMULATOR_BUILDDIR

cmake -DCMAKE_TOOLCHAIN_FILE=./ios-cmake/toolchain/iOS.cmake -DIOS_PLATFORM=SIMULATOR -GXcode ..

if (! xcodebuild -sdk iphonesimulator -configuration Release -target ALL_BUILD ARCHS="${ARCHS_SIM_32BIT}" ONLY_ACTIVE_ARCH=NO TARGET_BUILD_DIR="${SIM32}" ) ; then
    exit 1
fi

if (! xcodebuild -sdk iphonesimulator -configuration Release -target ALL_BUILD ARCHS="${ARCHS_SIM_64BIT}" ONLY_ACTIVE_ARCH=NO TARGET_BUILD_DIR="${SIM64}" ) ; then
    exit 1
fi

#===============================================================================
cd $SRCDIR
FRAMEWORK_NAME=`basename $1`

# sh framework_gen.sh $FRAMEWORK_NAME $IOS32 $SIM32 $1/include/$FRAMEWORK_NAME/
sh framework_gen.sh $FRAMEWORK_NAME $IOS64 $SIM64 $1/include/$FRAMEWORK_NAME/

echo "build_package : done !"
