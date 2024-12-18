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

set -e # exit on first error

export ROOT=`pwd`
export NCORES=`nproc`

function usage() {
    echo "Usage: ./BuildLinux.sh [-h][-u][-w][-r][-d][-s][-l][-t][-i][-v][-b debug|release][-g 2|3]"
    echo "   -h: this message"
    echo "   -u: only update dependency packets (optional and need sudo)"
    echo "   -w: wipe build directories before building"
    echo "   -g: force gtk2 build, need to then write the verison: '2' or '3' (BuildLinux.sh -g 2)"
    echo "   -b: build with debug symbols. Build in full debug mode if then followed by 'debug' (need to be lowercase)"
    echo "   -r: clean dependencies"
    echo "   -d: build deps"
    echo "   -s: build Slic3r"
    echo "   -l: update language .pot file"
    echo "   -t: build tests (in combination with -s)"
    echo "   -i: generate .tgz and appimage (optional)"
    echo "   -v: change the version 'UNKNOWN' to the date of the day"
    echo -e "\n   For a first use, you want to 'sudo ./BuildLinux.sh -u'"
    echo -e "   and then './BuildLinux.sh -dsi'\n"
}

function check_operating_system() {
# check operating system

    OS_FOUND=$( command -v uname)

    case   $( "${OS_FOUND}" | tr '[:upper:]' '[:lower:]') in
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

    echo
    if [ $TARGET_OS == "linux" ]; then
        if [ $(uname -m) == "x86_64" ]; then
            echo -e "$(tput setaf 2)Linux 64-bit found$(tput sgr0)\n"
            Processor="64"
        elif [[ $(uname -m) == "i386" || $(uname -m) == "i686" ]]; then
            echo "$(tput setaf 2)Linux 32-bit found$(tput sgr0)\n"
            Processor="32"
        else
            echo "$(tput setaf 1)Unsupported OS: Linux $(uname -m)"
            exit -1
        fi
    else
        echo -e "$(tput setaf 1)This script doesn't support your Operating system!"
        echo -e "Please use Linux 64-bit or Windows 10 64-bit with Linux subsystem / git-bash.$(tput sgr0)\n"
        exit -1
    fi
}

function check_available_memory_and_disk() {
    echo "Checking available memory and disk ..."
    FREE_MEM_GB=$(free -g -t | grep 'Mem:' | rev | cut -d" " -f1 | rev)
    MIN_MEM_GB=3
    echo "Available memory (GB): ${FREE_MEM_GB}"

    FREE_DISK_KB=$(df -k . | tail -1 | awk '{print $4}')
    MIN_DISK_KB=$((10 * 1024 * 1024))
    echo "Available disk (KB): ${FREE_DISK_KB}"

    if [ ${FREE_MEM_GB} -le ${MIN_MEM_GB} ]; then
        echo -e "\nERROR: SuperSlicer Builder requires at least ${MIN_MEM_GB}G of 'available' mem (systen has only ${FREE_MEM_GB}G available)"
        echo && free -h && echo
        exit 2
    fi

    if [[ ${FREE_DISK_KB} -le ${MIN_DISK_KB} ]]; then
        echo -e "\nERROR: SuperSlicer Builder requires at least $(echo ${MIN_DISK_KB} |awk '{ printf "%.1fG\n", $1/1024/1024; }') (systen has only $(echo ${FREE_DISK_KB} | awk '{ printf "%.1fG\n", $1/1024/1024; }') disk free)"
        echo && df -h . && echo
        exit 1
    fi
}

function check_distribution() {
    echo "Checking distribution ..."
    DISTRIBUTION=$(awk -F= '/^ID=/ {print $2}' /etc/os-release)
    # treat ubuntu as debian
    if [ "${DISTRIBUTION}" == "ubuntu" ]
    then
        DISTRIBUTION="debian"
    fi
    echo -e "$(tput setaf 2)${DISTRIBUTION} found$(tput sgr0)\n"
    if [ ! -f ./src/platform/unix/linux.d/${DISTRIBUTION} ]
    then
        echo "Your distribution does not appear to be currently supported by these build scripts"
        exit 1
    fi
}

#=======================================================================================

check_operating_system
check_distribution
check_available_memory_and_disk

#---------------------------------------------------------------------------------------
#check command line arguments
GTK_VERSION="3"
BUILD_MODE="release"
unset name
while getopts ":bdhilrstuvwg:" opt; do
    case ${opt} in
        u )
            UPDATE_LIB="1"
            ;;
        i )
            BUILD_IMAGE="1"
            ;;
        d )
            BUILD_DEPS="1"
            ;;
        s )
            BUILD_SLIC3R="1"
            ;;
        l )
            UPDATE_POTFILE="1"
            ;;
        t )
            BUILD_TESTS="1"
            ;;
        b )
            BUILD_DEBUG="1"
            BUILD_MODE=$OPTARG
            ;;
        g )
            GTK_VERSION=$OPTARG
            ;;
        r )
            BUILD_CLEANDEPEND="1"
            ;;
        v )
            VERSION_DATE="1"
            ;;
        w )
            BUILD_WIPE="1"
            ;;
        h ) usage
            exit 0
            ;;
        * ) usage
            exit -1
            ;;
    esac
done


if [ ${OPTIND} -eq 1 ]
then
    usage
    exit -1
fi

#---------------------------------------------------------------------------------------

# check installation of required packages or update when -u is set

source ./src/platform/unix/linux.d/${DISTRIBUTION}

if [[ "$GTK_VERSION" == 2 ]]; then
    FOUND_GTK2=$(dpkg -l libgtk* | grep gtk2)
    FOUND_GTK2_DEV=$(dpkg -l libgtk* | grep gtk2.0-dev)
    echo -e "\nFOUND_GTK2:\n$FOUND_GTK2\n"
    echo -e "\FOUND_GTK2_DEV:\n$FOUND_GTK2_DEV\n"
elif [[ "$GTK_VERSION" == 3 ]]; then
    FOUND_GTK3=$(dpkg -l libgtk* | grep gtk-3)
    FOUND_GTK3_DEV=$(dpkg -l libgtk* | grep gtk-3-dev)
    echo -e "\nFOUND_GTK3:\n$FOUND_GTK3)\n"
    echo -e "\nFOUND_GTK2_DEV:\n$FOUND_GTK3_DEV)\n"
else
    echo -e "\nError, only GTK2 and GTK3 verison are supported. Asked GTK version: '$GTK_VERSION'\n"
    exit -1
fi

if [[ -n "$BUILD_DEPS" ]]
then
    if [[ -n $BUILD_WIPE ]]
    then
       echo -e "\n wiping deps/build directory...\n"
       rm -fr deps/build
       echo -e " ... done\n"
    fi
    # mkdir build in deps
    if [ ! -d "deps/build" ]
    then
        mkdir deps/build
    fi
    echo -e "[1/9] Configuring dependencies ...\n"
    BUILD_ARGS=""
    if [[ -n "$FOUND_GTK3_DEV" ]]
    then
        BUILD_ARGS="-DDEP_WX_GTK3=ON"
    else
        BUILD_ARGS="-DDEP_WX_GTK3=OFF"
    fi
    if [[ -n "$BUILD_DEBUG" ]]; then
        if [[ $BUILD_MODE == "release" ]]; then
            BUILD_ARGS="${BUILD_ARGS} -DCMAKE_BUILD_TYPE=RelWithDebInfo"
        else
            BUILD_ARGS="${BUILD_ARGS} -DCMAKE_BUILD_TYPE=Debug"
        fi
        # have to build deps with debug & release or the cmake won't find evrything it needs
        if [ ! -d "deps/build/release" ]
        then
            mkdir deps/build/release
        fi
        pushd deps/build/release > /dev/null
        cmake ../.. -DDESTDIR="../destdir" $BUILD_ARGS
        popd > /dev/null
    fi

    pushd deps/build > /dev/null
    cmake .. $BUILD_ARGS
    echo -e "\n ... done\n"

    echo -e "\n[2/9] Building dependencies...\n"
    # make deps
    make -j$NCORES
    echo -e "\n ... done\n"

    # rename wxscintilla
    echo "[3/9] Renaming wxscintilla library..."
    pushd destdir/usr/local/lib  > /dev/null
    if [[ -z "$FOUND_GTK3_DEV" ]]
    then
        cp libwxscintilla-3.2.a libwx_gtk2u_scintilla-3.2.a
    else
        cp libwxscintilla-3.2.a libwx_gtk3u_scintilla-3.2.a
    fi
    echo "> ls destdir/usr/local/lib"
    ls -al .
    echo "> ls ROOT/deps/build/destdir/usr/local/lib"
    ls -al $ROOT/deps/build/destdir/usr/local/lib
    echo "> ls ROOT/deps/build"
    ls -al $ROOT/deps/build
    popd > /dev/null
    popd > /dev/null
    echo -e "\n ... done\n"
fi

if [[ -n "$BUILD_CLEANDEPEND" ]]
then
    echo -e "[4/9] Cleaning dependencies...\n"
    pushd deps/build > /dev/null
    df -h
    rm -fr dep_*
    rm -fr builds
    rm -fr download
    rm -fr release
    df -h
    popd > /dev/null
    echo -e " ... done\n"
fi

echo -n "Check space taken ..."
du -h

if [[ -n "$BUILD_SLIC3R" ]]
then
    echo -e "[5/9] Configuring SuperSlicer ...\n"
    if [[ -n $BUILD_WIPE ]]
    then
       echo -n "wiping build directory ..."
       rm -fr build
       echo " done"
    fi

	echo -n "Updating submodules ..."
	{
		# update submodule profiles
		pushd resources/profiles
		git submodule update --init
		popd
	} #> $ROOT/build/Build.log # Capture all command output
	echo " done"

	if [[ -n $VERSION_DATE ]]
    then
		echo -n "Changing date in version ..."
		# change date in version
		sed "s/+UNKNOWN/-$(date '+%F')/" version.inc > version.date.inc
		echo " done"
	else
		sed "s/+UNKNOWN//" version.inc > version.date.inc
    fi

    # mkdir build
    if [ ! -d "build" ]; then
        mkdir build
    fi

    BUILD_ARGS=""
    if [[ -n "$FOUND_GTK3_DEV" ]]; then
        BUILD_ARGS="-DSLIC3R_GTK=3"
    fi
    if [[ -n "$BUILD_DEBUG" ]]; then
        if [[ $BUILD_MODE == "release" ]]; then
            # add debug symbols (relwithdebinfo)
            BUILD_ARGS="${BUILD_ARGS} -DCMAKE_BUILD_TYPE=RelWithDebInfo"
            echo -n "Build in release mode but with debug symbols ..."
        else
            BUILD_ARGS="${BUILD_ARGS} -DCMAKE_BUILD_TYPE=Debug"
        fi
    fi

   if [[ -n "$BUILD_TESTS" ]]; then
       BUILD_ARGS="${BUILD_ARGS} -DCMAKE_BUILD_TESTS=1"
   else
       BUILD_ARGS="${BUILD_ARGS} -DCMAKE_BUILD_TESTS=0"
   fi

    # cmake
    pushd build > /dev/null
    cmake .. -DCMAKE_PREFIX_PATH="$PWD/../deps/build/destdir/usr/local" -DSLIC3R_STATIC=1 ${BUILD_ARGS}
    echo " ... done"
    # make SuperSlicer
    echo -e "\n[6/9] Building SuperSlicer ...\n"
    df -h
    make -j$NCORES libslic3r
    df -h
    du -h
    make -j$NCORES libslic3r_gui
    df -h
    du -h
    make -j$NCORES Slic3r
    df -h
    make -j$NCORES OCCTWrapper
    echo -e "\n ... done"

    echo -e "\n[7/9] Generating language files ...\n"
    #make .mo
    if [[ -n "$UPDATE_POTFILE" ]]
    then
        make gettext_make_pot
    fi
    make gettext_po_to_mo

    popd  > /dev/null
    echo -e "\n ... done"

    # Give proper permissions to script
    chmod 755 $ROOT/build/src/BuildLinuxImage.sh

    pushd build  > /dev/null
    $ROOT/build/src/BuildLinuxImage.sh -a -g $GTK_VERSION
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
    chmod 755 $ROOT/build/src/BuildLinuxImage.sh
    pushd build  > /dev/null
    $ROOT/build/src/BuildLinuxImage.sh -i -g $GTK_VERSION
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
