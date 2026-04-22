#! /bin/sh
# Script to download kernel files and construct associated metakernel

###############################################################################
###################### SET DESIRED GENERIC KERNELS HERE #######################
###############################################################################
leapsecond_kernels="naif0012.tls"
planet_kernels="a_old_versions/de421.bsp"
satellite_kernels="a_old_versions/mar097.bsp a_old_versions/jup344.bsp jup365.bsp sat415.bsp sat441.bsp a_old_versions/ura111l.bsp nep097.bsp a_old_versions/nep101.bsp plu060.bsp"
planetary_constants_kernels="pck00011.tpc Gravity.tpc"
###############################################################################
####################### END SET DESIRED GENERIC KERNELS #######################
###############################################################################

# get absolute path of script working directory
# needed due to the 'cd'ing that is done

KERNEL_FILE="kernels.txt" # name of metakernal file to create

GENERIC_KERNELS="pub/naif/generic_kernels"
SPICE_URL="http://naif.jpl.nasa.gov/"
add_kernel(){
    printf "                    '\$${1}/%s',\n" "${3}" >> ${KERNEL_FILE}
    wget -m -nH --cut-dirs=5 -e robots=off --trust-server-names -R 'index.html*'\
         -I "${GENERIC_KERNELS}/${2}/" -c "${SPICE_URL}${GENERIC_KERNELS}/${2}/${3}"\
         -P "${2}" -q --show-progress
}

# Move to Model/spice_kernels
BASEDIR=$(dirname $0) 
cd $BASEDIR

# if there is already a nonempty kernels.txt, move it to a backup and make an empty one
# THIS DOES NOT CHECK IF THERE ARE OTHER BACKUPS!!
if [ -s $KERNEL_FILE ]; then
    mv $KERNEL_FILE "kernels_bak.txt"
    touch $KERNEL_FILE
fi

# Headers/assign path values and symbols
printf "KPL/MK\n\\\begindata\n\n" >> $KERNEL_FILE
printf "PATH_VALUES = ( './Model/spice_kernels/lsk',
                './Model/spice_kernels/spk/planets',
                './Model/spice_kernels/spk/satellites',
                './Model/spice_kernels/pck' )\n\n" >> $KERNEL_FILE
printf "PATH_SYMBOLS = ( 'LSK', 'SPK_PLANETS', 'SPK_SATS', 'PCK' )\n\n" >> $KERNEL_FILE
printf "KERNELS_TO_LOAD = (\n" >> $KERNEL_FILE

# Download LSK files
mkdir lsk
for i in $leapsecond_kernels; do
    add_kernel LSK lsk $i
done

# Download SPK Planet files
mkdir spk
mkdir spk/planets
for i in $planet_kernels; do
    add_kernel SPK_PLANETS spk/planets $i
done

# Download SPK Satellite files
mkdir spk/satellites
for i in $satellite_kernels; do
    add_kernel SPK_SATS spk/satellites $i
done

# Download PCK files
mkdir pck
for i in $planetary_constants_kernels; do
    add_kernel PCK pck $i
    if [[ $i == "Gravity.tpc" ]]; then
        # Typo in J2 data for Luna. This corrects it
        unameOut="$(uname -s)"
        case "${unameOut}" in
            Darwin*)sed_cmd="sed -i '' ";;
            Linux*)sed_cmd="sed -i ";;
            *) echo "Unknown output for uname -s: $unameSOut"
        esac
        eval "${sed_cmd} '82s/[ \\]*begindata/      \\\begindata /' pck/${i}"
    fi
done

# End File
printf "                  )\n\n\\\begintext" >>  $KERNEL_FILE