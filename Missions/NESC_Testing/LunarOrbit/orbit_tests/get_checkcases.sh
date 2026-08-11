#! /bin/sh
# script to download the reference data for the lunar NESC sims from the website

###############################################################################
##################### SET CONFIGURATION INFORMATION HERE ######################
###############################################################################
TOP_URL="https://nescacademy.nasa.gov/workshop/FlightSim/2023"
SIM_IDS="1 2 3 4 5 5a 6 6a 7 8 8a 8b 8c 8d 9 9a 9b" 
REF_IDS="1 2 3 4 5 6 7 8"
###############################################################################
###################### END SET CONFIGURATION INFORMATION ######################
###############################################################################

# Move to NESC_Testing/LunarOrbit
BASEDIR=$(dirname $0) 
cd $BASEDIR

DAT_DIR="Orbital_checkcases"
mkdir $DAT_DIR
# example url for sim 8b, reference 1
# https://nescacademy.nasa.gov/workshop/FlightSim/2023/scn_8b/Lunar_08b_sim_01.csv
dl_refsim(){

    SIM_ID="${1}"
    REF_ID="${2}"
    OUT_DIR="${3}"
    REF_FILE_NAME="Lunar_0${SIM_ID}_sim_0${REF_ID}.csv"

    wget -m -nH -q --show-progress -e robots=off --trust-server-names -R 'index.html*'\
    -c "${TOP_URL}/scn_${SIM_ID}/${REF_FILE_NAME}" -O "${DAT_DIR}/${3}/${REF_FILE_NAME}"
}



for sim_id in $SIM_IDS; do
    out_dir="Lunar_0${sim_id^^}"
    mkdir "${DAT_DIR}/${out_dir}"

    # download all ref data for the sim in parallel
    for ref_id in $REF_IDS; do
        dl_refsim $sim_id $ref_id $out_dir &
    done
    # wait for the downloads to finish
    wait
done




