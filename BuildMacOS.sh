#!/bin/bash
#
# This script can download and compile dependencies, compile SuperSlicer
# and optional build a .tgz and an appimage.
#
# Original script from SuperSlicer by supermerill https://github.com/supermerill/SuperSlicer
#
# Change log:
#
# 20 Nov 2023, wschadow, branding and minor changes
# 01 Jan 2024, wschadow, added build options
#

export ROOT=`pwd`
export NCORES=`sysctl -n hw.ncpu`

OS_FOUND=$( command -v uname)

case $( "${OS_FOUND}" | tr '[:upper:]' '[:lower:]') in
  linux*)
    TARGET_OS="linux"
   ;;
  msys*|cygwin*|mingw*)
    # or possible 'bash on windows'
    TARGET_OS='windows'
   ;;
  nt|win*)
    TARGET_OS='windows'
    ;;
  darwin)
    TARGET_OS='macos'
    ;;
  *)
    TARGET_OS='unknown'
    ;;
esac

export TERM=xterm-256color 

# check operating system
BUILD_IMG_ARCH=""
echo
if [ $TARGET_OS == "macos" ]; then
    if [ $(uname -m) == "x86_64" ]; then
        echo -e "$(tput setaf 2)macOS x86_64 found$(tput sgr0)\n"
        Processor="x86_64"
        BUILD_IMG_ARCH="-x"
        echo "x86 BUILD_IMG_ARCH=${BUILD_IMG_ARCH}\n"
    elif [[ $(uname -m) == "i386" || $(uname -m) == "i686" ]]; then
        echo "$(tput setaf 2)macOS i386 / i686 (arm?) found$(tput sgr0)\n"
        Processor="arm64"
        BUILD_IMG_ARCH="-a"
    elif [ $(uname -m) == "arm64" ]; then
        echo "$(tput setaf 2)macOS arm64 found$(tput sgr0)\n"
        Processor="arm64"
        BUILD_IMG_ARCH="-a"
    else
        echo "$(tput setaf 1)Unsupported OS: macOS $(uname -m)"
        exit -1
    fi
else
    echo -e "$(tput setaf 1)This script doesn't support your Operating system!"
    echo -e "Please use a macOS.$(tput sgr0)\n"
    exit -1
fi
echo "BUILD_IMG_ARCH=${BUILD_IMG_ARCH}\n"

# Check if CMake is installed
export CMAKE_INSTALLED=`which cmake`
if [[ -z "$CMAKE_INSTALLED" ]]
then
    echo "Can't find CMake. Either is not installed or not in the PATH. Aborting!"
    exit -1
fi

while getopts ":idaxbhcsltwrv" opt; do
  case ${opt} in
    i )
        BUILD_IMAGE="1"
        ;;
    d )
        BUILD_DEPS="1"
        ;;
    a )
        BUILD_ARCH="arm64"
        BUILD_IMG_ARCH="-a"
        ;;
    x )
        BUILD_ARCH="x86_64"
        BUILD_IMG_ARCH="-x"
        ;;
    b )
        BUILD_DEBUG="1"
        ;;
    s )
        BUILD_SLIC3R="1"
        ;;
    t)
        BUILD_TESTS="1"
        ;;
    l )
        UPDATE_POTFILE="1"
        ;;
    c)
        BUILD_XCODE="1"
        ;;
	v )
		VERSION_DATE="1"
		;;
    w )
        BUILD_WIPE="1"
        ;;
    r )
        BUILD_CLEANDEPEND="1"
        ;;
    h ) echo "Usage: ./BuildMacOS.sh [-h][-w][-d][-r][-a][-x][-b][-c][-s][-t][-i]"
        echo "   -h: this message"
        echo "   -w: wipe build directories before building"
        echo "   -d: build dependencies"
        echo "   -r: clean dependencies building files (reduce disk usage)"
        echo "   -a: build for arm64 (Apple Silicon)"
        echo "   -x: build for x86_64 (Intel)"
        echo "   -b: build with debug symbols"
        echo "   -c: build for XCode"
        echo "   -s: build Slic3r/SuperSlicer"
        echo "   -t: build tests (in combination with -s)"
        echo "   -i: generate DMG image (optional)\n"
		echo "   -v: change the version 'UNKNOWN' to the date of the day"
        exit 0
        ;;
  esac
done

if [ $OPTIND -eq 1 ]
then
    echo "Usage: ./BuildLinux.sh [-h][-w][-d][-r][-a][-x][-b][-c][-s][-t][-i]"
    echo "   -h: this message"
    echo "   -w: wipe build directories before building"
    echo "   -d: build dependencies"
    echo "   -r: clean dependencies building files (reduce disk usage)"
    echo "   -a: build for arm64 (Apple Silicon)"
    echo "   -x: build for x86_64 (Intel)"
    echo "   -b: build with debug symbols"
    echo "   -c: build for XCode"
    echo "   -s: build Slic3r/SuperSlicer"
    echo "   -t: build tests (in combination with -s)"
    echo -e "   -i: Generate DMG image (optional)\n"
    exit 0
fi

echo "Build architecture: ${BUILD_ARCH}"
echo "Build IMG_ARCH=${BUILD_IMG_ARCH}\n"

echo "\n/Applications:\n"
ls /Applications
echo "\n/Applications/Xcode_13.2.1.app:\n"
ls /Applications/Xcode_13.2.1.app
echo "\n/Applications/Xcode_13.2.1.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs:\n"
ls /Applications/Xcode_13.2.1.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs
echo "\n/Applications/Xcode_13.2.1.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX12.1.sdk/usr/lib:\n"
ls /Applications/Xcode_13.2.1.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX12.1.sdk/usr/lib

# Iconv: /Applications/Xcode_13.2.1.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX12.1.sdk/usr/lib/libiconv.tbd
echo "\nbrew --prefix libiconv:\n"
brew --prefix libiconv
echo "\nbrew --prefix zstd:\n"
brew --prefix zstd
export LIBRARY_PATH=$LIBRARY_PATH:$(brew --prefix zstd)/lib/
# not enough to fix the issue on cross-compiling
#if [[ -n "$BUILD_ARCH" ]]
#then
#    export LIBRARY_PATH=$LIBRARY_PATH:$(brew --prefix libiconv)/lib/
#fi

export $BUILD_ARCH
export LIBRARY_PATH=$LIBRARY_PATH:$(brew --prefix zstd)/lib/

echo -n "[1/9] Updating submodules..."
{
    # update submodule profiles
    pushd resources/profiles
    git submodule update --init
    popd
} #> $ROOT/build/Build.log # Capture all command output
echo "done"


if [[ -n "$VERSION_DATE" ]]
then
	echo -n "[2/9] Changing date in version ... "
    # change date in version
    sed "s/+UNKNOWN/-$(date '+%F')/" version.inc > version.date.inc
	echo "done"
else
	echo -n "[2/9] Changing date in version: remove UNKNOWN ... "
    sed "s/+UNKNOWN//" version.inc > version.date.inc
	echo "done"
fi

if [[ -n "$BUILD_DEPS" ]]
then
    if [[ -n $BUILD_WIPE ]]
    then
       echo -e "\n wiping deps/build directory ...\n"
       rm -fr deps/build
       echo -e " ... done\n"
    fi
    # mkdir in deps
    if [ ! -d "deps/build" ]
    then
        mkdir deps/build
    fi
    echo -e " \n[3/9] Configuring dependencies ... \n"
    BUILD_ARGS=""
    if [[ -n "$BUILD_ARCH" ]]
    then
        BUILD_ARGS="${BUILD_ARGS}  -DCMAKE_OSX_ARCHITECTURES:STRING=${BUILD_ARCH}"
    fi
    if [[ -n "$BUILD_DEBUG" ]]
    then
        BUILD_ARGS="${BUILD_ARGS} -DCMAKE_BUILD_TYPE=Debug"
    fi
    # cmake deps
    echo "Cmake command: cmake .. -DCMAKE_OSX_DEPLOYMENT_TARGET=\"10.15\" ${BUILD_ARCH} "
    pushd deps/build > /dev/null
    cmake .. -DCMAKE_OSX_DEPLOYMENT_TARGET="10.15" $BUILD_ARGS

    echo -e "\n ... done\n"

    echo -e "[4/9] Building dependencies ...\n"

    # make deps
    make -j$NCORES

    echo -e "\n ... done\n"

    echo -e "[5/9] Renaming wxscintilla library ...\n"

    # rename wxscintilla
    pushd destdir/usr/local/lib > /dev/null
    cp libwxscintilla-3.2.a libwx_osx_cocoau_scintilla-3.2.a

    popd > /dev/null
    popd > /dev/null
    echo -e "\n ... done\n"
fi

if [[ -n "$BUILD_CLEANDEPEND" ]]
then
    echo -e "[6/9] Cleaning dependencies...\n"
    pushd deps/build
    pwd
    rm -fr dep_*
    popd > /dev/null
    echo -e "\n ... done\n"
fi

if [[ -n "$BUILD_SLIC3R" ]]
then
    echo -e "[5/9] Configuring Slicer ...\n"

    if [[ -n $BUILD_WIPE ]]
    then
       echo -e "\n wiping build directory...\n"
       rm -fr build
       echo -e " ... done\n"
    fi

    # mkdir build
    if [ ! -d "build" ]
    then
        mkdir build
    fi

    BUILD_ARGS=""
    if [[ -n "$BUILD_ARCH" ]]
    then
        BUILD_ARGS="${BUILD_ARGS} -DCMAKE_OSX_ARCHITECTURES=${BUILD_ARCH}"
    fi
    if [[ -n "$BUILD_DEBUG" ]]
    then
        BUILD_ARGS="-DCMAKE_BUILD_TYPE=Debug ${BUILD_ARGS}"
    fi
    if [[ -n "$BUILD_XCODE" ]]
    then
        BUILD_ARGS="-GXcode ${BUILD_ARGS}"
    fi

    if [[ -n "$BUILD_TESTS" ]]
    then
        BUILD_ARGS="${BUILD_ARGS} -DCMAKE_BUILD_TESTS=1"
    else
        BUILD_ARGS="${BUILD_ARGS} -DCMAKE_BUILD_TESTS=0"
    fi

    # cmake
    pushd build > /dev/null
    cmake .. -DCMAKE_PREFIX_PATH="$PWD/../deps/build/destdir/usr/local" -DCMAKE_OSX_DEPLOYMENT_TARGET="10.14" -DSLIC3R_STATIC=1 ${BUILD_ARGS}
    echo -e "\n ... done"

    # make Slic3r
    if [[ -z "$BUILD_XCODE" ]]
    then
        echo -e "\n[6/9] Building Slicer ...\n"
        make -j$NCORES
        echo -e "\n ... done"
    fi

    echo -e "\n[7/9] Generating language files ...\n"
    #make .mo
    if [[ -n "$UPDATE_POTFILE" ]]
    then
        make gettext_make_pot
    fi
    make gettext_po_to_mo

    popd  > /dev/null
    echo -e "\n ... done"
    popd  > /dev/null

    echo "> ls ROOT"
    ls -al $ROOT
    echo "> ls ROOT/build"
    ls -al $ROOT/build
    echo "> ls -al ROOT/build/bin"
    ls -al $ROOT/build/bin
    echo "> ls -al ROOT/build/src"
    ls -al $ROOT/build/src
fi

if [[ -n "$BUILD_IMAGE" ]]
then
    # Give proper permissions to script
    chmod 755 $ROOT/build/src/BuildMacOSImage.sh
    pushd build  > /dev/null
    echo "> $ROOT/build/src/BuildMacOSImage.sh -i ${BUILD_IMG_ARCH}"
    $ROOT/build/src/BuildMacOSImage.sh -i $BUILD_IMG_ARCH
    popd  > /dev/null
    echo "> ls ROOT"
    ls -al $ROOT
    echo "> ls ROOT/build"
    ls -al $ROOT/build
    echo "> ls -al ROOT/build/bin"
    ls -al $ROOT/build/bin
    echo "> ls -al ROOT/build/bin"
    ls -al $ROOT/build/bin
    echo "> ls -al ROOT/build/src"
    ls -al $ROOT/build/src
fi
