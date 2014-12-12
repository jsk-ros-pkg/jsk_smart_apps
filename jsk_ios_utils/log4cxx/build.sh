#!/bin/sh -x

#===============================================================================

: ${APR:="apr-1.4.6"}
: ${APR_UTIL:="apr-util-1.5.2"}
: ${LOG4CXX:="apache-log4cxx-0.10.0"}

#===============================================================================

: ${SRCDIR:=`pwd`}
: ${OS_BUILDDIR=`pwd`/iPhoneOS_build}
: ${SIMULATOR_BUILDDIR=`pwd`/iPhoneSimulator_build}

#===============================================================================

if [ ! -d ios-cmake ]
    then
#    hg clone https://code.google.com/p/ios-cmake/
    git clone https://github.com/furushchev/ios-cmake.git
fi

#===============================================================================
curl http://archive.apache.org/dist/apr/$APR.tar.gz -o ./$APR.tar.gz
curl http://archive.apache.org/dist/apr/$APR_UTIL.tar.gz -o ./$APR_UTIL.tar.gz
curl http://archive.apache.org/dist/logging/log4cxx/0.10.0/$LOG4CXX.tar.gz -o ./$LOG4CXX.tar.gz

echo "Extracting ..."

[ -d $APR ] && rm -rf $APR
[ -d $APR_UTIL ] && rm -rf $APR_UTIL
[ -d $LOG4CXX ] && rm -rf $LOG4CXX

tar xvzf $APR.tar.gz
tar xvzf $APR_UTIL.tar.gz
tar xvzf $LOG4CXX.tar.gz

#===============================================================================
echo "Configuring ..."

cd $SRCDIR/$APR
./configure --without-sendfile

cd $SRCDIR/$APR_UTIL
./configure --with-apr="../$APR/" --without-pgsql --without-mysql --without-sqlite2 --without-sqlite3 --without-oracle --without-freetds --without-odbc

cd $SRCDIR/$APR_UTIL/xml/expat/
./configure

cd $SRCDIR/$LOG4CXX
./configure --with-apr="../$APR/"

#===============================================================================
echo "Patching ..."

patch -N $SRCDIR/$APR/include/apr_general.h $SRCDIR/patches/apr_general.patch
patch -N $SRCDIR/$APR/include/apr.h $SRCDIR/patches/apr.patch
patch -N $SRCDIR/$APR_UTIL/xml/expat/expat_config.h $SRCDIR/patches/expat_config.patch
patch -N $SRCDIR/$APR_UTIL/xml/expat/lib/xmlparse.c $SRCDIR/patches/xmlparse.patch
patch -N $SRCDIR/$LOG4CXX/src/main/include/log4cxx/helpers/simpledateformat.h $SRCDIR/patches/simpledateformat.patch
patch -N $SRCDIR/$LOG4CXX/src/main/cpp/stringhelper.cpp $SRCDIR/patches/stringhelper.patch
patch -N $SRCDIR/$APR/file_io/unix/readwrite.c $SRCDIR/patches/readwrite.c.patch

#===============================================================================
echo "Generating CMakeLists.txt ..."

cd $SRCDIR

cat > CMakeLists.txt <<EOF

cmake_minimum_required(VERSION 2.8.0)

project($LOG4CXX)

include_directories(
    ./$APR/include/
    ./$APR/include/arch
    ./$APR/include/arch/unix

    ./$APR_UTIL/include
    ./$APR_UTIL/include/private
    ./$APR_UTIL/xml/expat
    ./$APR_UTIL/xml/expat/lib

    ./$LOG4CXX/src/main/include
)

add_library($LOG4CXX STATIC
$(find ./$APR -name \*.c | grep -v 'test' | grep 'unix\|tables\|string\|passwd')

$(find ./$APR_UTIL -name \*.c ! -name xmltok_impl.c ! -name xmltok_ns.c | grep -v 'test')

$(find ./$LOG4CXX -name \*.cpp | grep -v 'test' | grep -v 'examples')
)
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

# build for ios 32bit
if (! xcodebuild -configuration Release -target ALL_BUILD -sdk iphoneos ARCHS="${ARCHS_IOS_32BIT}" ONLY_ACTIVE_ARCH=NO clean build TARGET_BUILD_DIR="${IOS32}")
    then
        exit 1
fi

# build for ios 64bit
if (! xcodebuild -configuration Release -target ALL_BUILD -sdk iphoneos ARCHS="${ARCHS_IOS_64BIT}" ONLY_ACTIVE_ARCH=NO clean build TARGET_BUILD_DIR="${IOS64}")
    then
        exit 1
fi


cd $SIMULATOR_BUILDDIR

cmake -DCMAKE_TOOLCHAIN_FILE=./ios-cmake/toolchain/iOS.cmake -DIOS_PLATFORM=SIMULATOR -GXcode ..

# build for simulator 32bit
if (! xcodebuild -configuration Release -target ALL_BUILD -sdk iphonesimulator ARCHS="${ARCHS_SIM_32BIT}" ONLY_ACTIVE_ARCH=NO clean build TARGET_BUILD_DIR="${SIM32}")
    then
        exit 1
fi

# build for simulator 64bit
if (! xcodebuild -configuration Release -target ALL_BUILD -sdk iphonesimulator ARCHS="${ARCHS_SIM_64BIT}" ONLY_ACTIVE_ARCH=NO clean build TARGET_BUILD_DIR="${SIM64}")
    then
        exit 1
fi

#===============================================================================
cd $SRCDIR

VERSION_TYPE=Alpha
FRAMEWORK_NAME=log4cxx
FRAMEWORK_VERSION=A

FRAMEWORK_CURRENT_VERSION=$LOG4CXX
FRAMEWORK_COMPATIBILITY_VERSION=$LOG4CXX

FRAMEWORK_BUNDLE=$SRCDIR/$FRAMEWORK_NAME.framework
echo "Framework: Building $FRAMEWORK_BUNDLE ..."

[ -d $FRAMEWORK_BUNDLE ] && rm -rf $FRAMEWORK_BUNDLE

echo "Framework: Setting up directories..."
mkdir -p $FRAMEWORK_BUNDLE
mkdir -p $FRAMEWORK_BUNDLE/Versions
mkdir -p $FRAMEWORK_BUNDLE/Versions/$FRAMEWORK_VERSION
mkdir -p $FRAMEWORK_BUNDLE/Versions/$FRAMEWORK_VERSION/Resources
mkdir -p $FRAMEWORK_BUNDLE/Versions/$FRAMEWORK_VERSION/Headers
mkdir -p $FRAMEWORK_BUNDLE/Versions/$FRAMEWORK_VERSION/Documentation
touch $FRAMEWORK_BUNDLE/Versions/$FRAMEWORK_VERSION/Documentation/.gitkeep

echo "Framework: Creating symlinks..."
ln -s $FRAMEWORK_VERSION               $FRAMEWORK_BUNDLE/Versions/Current
ln -s Versions/Current/Headers         $FRAMEWORK_BUNDLE/Headers
ln -s Versions/Current/Resources       $FRAMEWORK_BUNDLE/Resources
ln -s Versions/Current/Documentation   $FRAMEWORK_BUNDLE/Documentation
ln -s Versions/Current/$FRAMEWORK_NAME $FRAMEWORK_BUNDLE/$FRAMEWORK_NAME

FRAMEWORK_INSTALL_NAME=$FRAMEWORK_BUNDLE/Versions/$FRAMEWORK_VERSION/$FRAMEWORK_NAME

echo "Lipoing library into $FRAMEWORK_INSTALL_NAME..."
#lipo -create $SIM32/lib$LOG4CXX.a $IOS32/lib$LOG4CXX.a -output $FRAMEWORK_INSTALL_NAME
lipo -create $SIM64/lib$LOG4CXX.a $IOS64/lib$LOG4CXX.a -output $FRAMEWORK_INSTALL_NAME

echo "Framework: Copying includes..."

cp $SRCDIR/$APR/include/*.h $FRAMEWORK_BUNDLE/Headers/
mkdir $FRAMEWORK_BUNDLE/Headers/arch
cp $SRCDIR/$APR/include/arch/*.h $FRAMEWORK_BUNDLE/Headers/arch
mkdir $FRAMEWORK_BUNDLE/Headers/arch/unix
cp $SRCDIR/$APR/include/arch/unix/*.h $FRAMEWORK_BUNDLE/Headers/arch/unix

cp $SRCDIR/$APR_UTIL/include/*.h $FRAMEWORK_BUNDLE/Headers/
mkdir $FRAMEWORK_BUNDLE/Headers/private
cp $SRCDIR/$APR_UTIL/include/private/*.h $FRAMEWORK_BUNDLE/Headers/private
mkdir $FRAMEWORK_BUNDLE/Headers/expat
cp $SRCDIR/$APR_UTIL/xml/expat/*.h $FRAMEWORK_BUNDLE/Headers/expat
mkdir $FRAMEWORK_BUNDLE/Headers/expat/lib
cp $SRCDIR/$APR_UTIL/xml/expat/lib/*.h $FRAMEWORK_BUNDLE/Headers/expat/lib

cp -r $SRCDIR/$LOG4CXX/src/main/include/log4cxx/* $FRAMEWORK_BUNDLE/Headers/

echo "Framework: Creating plist..."

cat > $FRAMEWORK_BUNDLE/Resources/Info.plist <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
    <dict>
        <key>CFBundleDevelopmentRegion</key>
        <string>English</string>
        <key>CFBundleExecutable</key>
        <string>${FRAMEWORK_NAME}</string>
        <key>CFBundleIdentifier</key>
        <string>apache</string>
        <key>CFBundleInfoDictionaryVersion</key>
        <string>6.0</string>
        <key>CFBundlePackageType</key>
        <string>FMWK</string>
        <key>CFBundleSignature</key>
        <string>????</string>
        <key>CFBundleVersion</key>
        <string>${FRAMEWORK_CURRENT_VERSION}</string>
    </dict>
</plist>
EOF

echo "Done !"
