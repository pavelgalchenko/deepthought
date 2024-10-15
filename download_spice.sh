#! /bin/sh
# script to download kernel files

SPICE_URL="https://naif.jpl.nasa.gov/pub/naif/toolkit//C/"


unameSOut="$(uname -s)"
unameMOut="$(uname -m)"
case "${unameSOut}" in
    Darwin*)
    case "${unameMOut}" in
        arm64)sub_url="MacM1_OSX_clang_64bit";;
        x86_64)sub_url="MacIntel_OSX_AppleC_64bit";;
        *)echo "For uname -s=Darwin, invalid/unknown output for uname -m : ${unameMOut}";;
    esac;;
    Linux*)
    case "${unameMOut}" in
        x86_64)sub_url="PC_Linux_GCC_64bit";;
        i686* | i386)sub_url="PC_Linux_GCC_32bit";;
        *)echo "For uname -s=Linux, invalid/unknown output for uname -m : ${unameMOut}";;
    esac;;
    *) echo "Invalid/unknown output for uname -s: $unameSOut";;
esac


BASEDIR=$(dirname $0)
cd "$BASEDIR"

wget "${SPICE_URL}${sub_url}/packages/cspice.tar.Z"
tar xfv cspice.tar.Z

rm cspice.tar.Z