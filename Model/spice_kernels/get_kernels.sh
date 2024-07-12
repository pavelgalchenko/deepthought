#! /bin/sh
# script to download kernel files

GENERIC_KERNELS="pub/naif/generic_kernels"
SPICE_URL="http://naif.jpl.nasa.gov/"
wget_kernels(){
    wget -m -nH --cut-dirs=5 -e robots=off --trust-server-names -R 'index.html*' -I "${GENERIC_KERNELS}/${1}/${2}" -nv "${SPICE_URL}${GENERIC_KERNELS}/${1}/${2}"
}

leapsecond_kernels="naif0012.tls"
planet_kernels="de440.bsp de430.bsp"
satellite_kernels="mar097.bsp jup344.bsp jup365.bsp sat415.bsp sat441.bsp ura111l.bsp nep097.bsp a_old_versions/nep101.bsp plu060.bsp"
planetary_constants_kernels="pck00010.tpc Gravity.tpc"

BASEDIR=$(dirname $0)
cd "$BASEDIR"

mkdir spk
mkdir spk/planets
cd spk/planets
for i in $planet_kernels; do
    wget_kernels spk/planets $i
done

mkdir ../satellites
cd ../satellites
for i in $satellite_kernels; do
    wget_kernels spk/satellites $i
done

mkdir ../../pck
cd ../../pck
for i in $planetary_constants_kernels; do
    wget_kernels pck $i
done

mkdir ../lsk
cd ../lsk
for i in $leapsecond_kernels; do
    wget_kernels lsk $i
done