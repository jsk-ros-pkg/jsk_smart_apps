SRCDIR=`pwd`
LOGFILE=$SRCDIR/log.txt

if [ -d $SRCDIR/boost/libs/boost ]; then
    rm -rf $SRCDIR/boost
    git clone https://github.com/danoli3/ofxiOSBoost.git boost
fi

BOOST_ROOT=$SRCDIR/boost/libs/boost
BOOST_LIBS=$BOOST_ROOT/ios
BOOST_INCLUDE=$BOOST_ROOT/include/boost

unifyBoost()
{
    ARCH=$1
    echo "unifying *.a to libboost.a"
    pushd $BOOST_LIBS/$ARCH
    if [ -d obj ]; then
        rm -rf obj
    fi
    mkdir obj
    ls | grep .a$ | xargs -I{} ar x {}
    mv *.o obj
    ar crus $BOOST_LIBS/libboost-$ARCH.a obj/*.o
    popd
}

makeInfoPlist()
{
    cat <<EOF > $1/Info.plist
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleDevelopmentRegion</key>
    <string>English</string>
    <key>CFBundleExecutable</key>
    <string>boost</string>
    <key>CFBundleIdentifier</key>
    <string>org.boost</string>
    <key>CFBundleInfoDictionaryVersion</key>
    <string>6.0</string>
    <key>CFBundlePackageType</key>
    <string>FMWK</string>
    <key>CFBundleSignature</key>
    <string>????</string>
    <key>CFBundleVersion</key>
    <string>1.54.0</string>
</dict>
</plist>
EOF
}

makeFramework()
{
    A=$SRCDIR/boost.framework/Versions/A
    if [ -d $SRCDIR/boost.framework ]; then
        rm -rf $SRCDIR/boost.framework
    fi
    mkdir -p $A/Documentation $A/Headers $A/Resources
    cp $BOOST_LIBS/boost.a $A/boost
    cp -R $BOOST_INCLUDE/* $A/Headers
    makeInfoPlist $A/Resources
    (cd $A/..; ln -s A Current)
    (cd $SRCDIR/boost.framework; ln -s Versions/Current/Documentation Documentation )
    (cd $SRCDIR/boost.framework; ln -s Versions/Current/Headers Headers )
    (cd $SRCDIR/boost.framework; ln -s Versions/Current/Resources Resources )
    (cd $SRCDIR/boost.framework; ln -s Versions/Current/boost boost )
}
makeFramework
