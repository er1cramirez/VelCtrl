#!/bin/bash
NB_THREADS=$(nproc)
from_scratch=no
IDE_SCRIPT=cmake_codelite_outofsource.sh
toolchains=($OECORE_CMAKE_TOOLCHAINS)

function notify () {
	notify=$(which notify-send)
    if [ "$notify" != "" ]; then
	    notify-send Flair "$1" $2 $3
    fi
}

function notify_ok () {
	notify "$1" -i emblem-default
}

function notify_ko () {
	notify "$1" -i error
}


function green_echo () {
	echo -e "\033[32m$1\033[0m"
}

function red_echo () {
	echo -e "\033[31m$1\033[0m"
}

function check_error () {
	if [ "$?" != "0" ]; then
		red_echo "Error, exiting"
        notify_ko "error while compiling" 
		exit 1
	fi
}

function sanity_check () {
	if [ -z $FLAIR_ROOT ]; then
		red_echo "You must set the FLAIR_ROOT environement variable"
		exit 1
	fi

    if ! [ -v OECORE_CMAKE_TOOLCHAINS ]; then
		red_echo "You must install a flair toolchain"
		exit 1
	fi

    if ! [ -d $FLAIR_ROOT/flair-build ]; then
		green_echo "Creating $FLAIR_ROOT/flair-build directory"
        mkdir -p $FLAIR_ROOT/flair-build
	fi
}

sanity_check

printf "Compile all from scratch (flair-build directory will be erased) [Y/n]?"
read answer

if [ "$answer" = "" ] || [ "$answer" = "y" ] || [ "$answer" = "Y" ]; then
    notify "started configuration"	
    rm -rf $FLAIR_ROOT/flair-build/*
    cd $FLAIR_ROOT/flair-build
    $FLAIR_ROOT/flair-src/scripts/$IDE_SCRIPT $FLAIR_ROOT/flair-src/
fi

#iterate over available toolchains
for arch in ${toolchains[@]}; do
    green_echo "Compiling and installing for $arch"
    notify "compiling and installing for $arch"
    cd $FLAIR_ROOT/flair-build/build_$arch
    make -j$NB_THREADS
    check_error	
    make install
done


printf "Compile Flair libs documentation [Y/n]?"
read answer

if [ "$answer" = "" ] || [ "$answer" = "y" ] || [ "$answer" = "Y" ]; then
    #get doxygen, $arch is the last used from previous for and should be valid
    DOXYGEN=$(eval "echo \"\$OECORE_${arch^^}_NATIVE_SYSROOT\"")/usr/bin/doxygen
	$DOXYGEN $FLAIR_ROOT/flair-src/lib/Doxyfile.in
fi

notify_ok "finished compilation" 

exit 0
