# BEGINCOMMENT
#
#
#
#
# ENDCOMMENT

#################################
# BEGIN USER PLOTTING FUNCTIONS #
#################################

###############################################################################
# STOP STOP STOP STOP STOP STOP STOP  STOP STOP STOP STOP STOP STOP STOP STOP #
###############################################################################

# NO FURTHER USER EDITABLE PARAMETERS, EDITING CODE BELOW SHALL DESTROY WORLD #

###############################################################################
# STOP STOP STOP STOP STOP STOP STOP  STOP STOP STOP STOP STOP STOP STOP STOP #
###############################################################################

# Required Packages

# %%
import numbers
import numpy as np
import sys
import os
import re
import ruamel.yaml
from ruamel.yaml import YAML
from parse import parse
import copy
import ruamel.yaml.comments


# %%
def convertSC(missionDir, yaml, fileName, commentDict=None):
    scName = fileName[3:-4]
    scDict = dict()
    scDict["Name"] = scName
    configDict = scDict["Configuration"] = dict()
    orbDict = scDict["Orbit"] = dict()
    attDict = scDict["Attitude"] = dict()
    dynDict = scDict["Dynamics Flags"] = dict()
    scDict["Bodies"] = list()
    scDict["Joints"] = list()
    scDict["Wheel Params"] = dict()
    scDict["Wheels"] = list()
    scDict["MTBs"] = list()
    scDict["Thrusters"] = list()
    scDict["Gyros"] = list()
    scDict["Magnetometers"] = list()
    scDict["CSSs"] = list()
    scDict["FSSs"] = list()
    scDict["STs"] = list()
    scDict["GPSs"] = list()
    scDict["Accelerometers"] = list()
    with open(missionDir + "/InOut/" + fileName, "r") as f:
        lines = f.readlines()

    ### Default Information
    for lineNum, line in enumerate(lines[0:25]):
        lineData = line.split("!")[0].strip()
        strData = lineData.split()
        match lineNum:
            case 1:
                configDict["Description"] = lineData
            case 2:
                configDict["Label"] = lineData.strip('"')
            case 3:
                configDict["Sprite File"] = lineData
            case 4:
                configDict["FSW Identifier"] = lineData
            case 5:
                configDict["FSW Sample Time"] = float(lineData)
            case 7:
                orbDict["Prop Type"] = lineData
            case 8:
                orbDict["Pos Specifier"] = lineData
            case 9:
                orbDict["Pos wrt F"] = [float(string) for string in strData]
            case 10:
                orbDict["Vel wrt F"] = [float(string) for string in strData]
            case 12:
                attDict["Ang Vel Frame"] = lineData[0]
                attDict["Att Representation"] = lineData[1]
                attDict["Att Frame"] = lineData[2]
            case 13:
                attDict["Ang Vel"] = [float(string) for string in strData]
            case 14:
                attDict["Quaternion"] = [float(string) for string in strData]
            case 15:
                attDict["Euler Angles"] = dict()
                attDict["Euler Angles"]["Angles"] = [
                    float(string) for string in strData[0:3]
                ]
                attDict["Euler Angles"]["Sequence"] = int(strData[3])
            case 17:
                dynDict["Method"] = lineData
            case 18:
                dynDict["Compute Constraints"] = lineData.lower() == "true"
            case 19:
                dynDict["Mass Reference Point"] = lineData
            case 20:
                dynDict["Flex Active"] = lineData.lower() == "true"
            case 21:
                dynDict["2nd Order Flex"] = lineData.lower() == "true"
            case 22:
                dynDict["Shaker File Name"] = lineData
            case 23:
                dynDict["Drag Coefficient"] = float(lineData)

    ### Bodies
    nItem = int(lines[27].split("!")[0].strip())
    offset = 28
    secLength = yaml_data["lngth_body"]
    for itemNum in range(nItem):
        newDict = dict()
        newItem = {"Body": newDict}
        newDict["Index"] = itemNum
        for lineNum, line in enumerate(lines[offset + 0 : offset + secLength]):
            lineData = line.split("!")[0].strip()
            strData = lineData.split()
            match lineNum:
                case 1:
                    newDict["Mass"] = float(lineData)
                case 2:
                    newDict["MOI"] = [float(string) for string in strData]
                case 3:
                    newDict["POI"] = [float(string) for string in strData]
                case 4:
                    newDict["Pos of CM"] = [float(string) for string in strData]
                case 5:
                    newDict["Constant Momentum"] = [float(string) for string in strData]
                case 6:
                    newDict["Constant Dipole"] = [float(string) for string in strData]
                case 7:
                    newDict["Geometry File Name"] = lineData
                case 8:
                    newDict["Node File Name"] = lineData
                case 9:
                    newDict["Flex File Name"] = lineData
        offset += secLength
        scDict["Bodies"].append(newItem)

    ### Joints
    nItem -= 1
    offset += yaml_data["lngth_header"] + 1
    secLength = yaml_data["lngth_joint"]

    if nItem == 0:
        offset += secLength
    else:
        for itemNum in range(nItem):
            newDict = dict()
            newItem = {"Joint": newDict}
            newDict["Index"] = itemNum
            newDict["Body Indicies"] = list()
            for lineNum, line in enumerate(lines[offset + 0 : offset + secLength]):
                lineData = line.split("!")[0].strip()
                strData = lineData.split()
                match lineNum:
                    case 1:
                        newDict["Joint Type"] = lineData
                    case 2:
                        newDict["Body Indicies"] = [int(string) for string in strData]
                    case 3:
                        newDict["Rot DOF"] = int(strData[0])
                        newDict["Rot Sequence"] = int(strData[1])
                        newDict["Rot Type"] = strData[2]
                    case 4:
                        newDict["Trn DOF"] = int(strData[0])
                        newDict["Trn Sequence"] = int(strData[1])
                    case 5:
                        newDict["Rot DOF Locked"] = [
                            string.lower() == "true" for string in strData
                        ]
                    case 6:
                        newDict["Trn DOF Locked"] = [
                            string.lower() == "true" for string in strData
                        ]
                    case 7:
                        newDict["Init Angles"] = [float(string) for string in strData]
                    case 8:
                        newDict["Init Angle Rates"] = [
                            float(string) for string in strData
                        ]
                    case 9:
                        newDict["Init Displacement"] = [
                            float(string) for string in strData
                        ]
                    case 10:
                        newDict["Init Displacement Rates"] = [
                            float(string) for string in strData
                        ]
                    case 11:
                        newDict["Bi-Gi Angles"] = dict()
                        newDict["Bi-Gi Angles"]["Angles"] = [
                            float(string) for string in strData[0:3]
                        ]
                        newDict["Bi-Gi Angles"]["Sequence"] = int(strData[3])
                    case 12:
                        newDict["Bo-Go Angles"] = dict()
                        newDict["Bo-Go Angles"]["Angles"] = [
                            float(string) for string in strData[0:3]
                        ]
                        newDict["Bo-Go Angles"]["Sequence"] = int(strData[3])
                    case 13:
                        newDict["Pos wrt Inner Body"] = [
                            float(string) for string in strData
                        ]
                    case 14:
                        newDict["Pos wrt Outer Body"] = [
                            float(string) for string in strData
                        ]
                    case 15:
                        newDict["Parm File Name"] = lineData
            offset += secLength
            scDict["Joints"].append(newItem)

    ### Wheels
    scDict["Wheel Params"]["Drag"] = (
        lines[offset + 1].split("!")[0].strip().lower() == "true"
    )
    scDict["Wheel Params"]["Jitter"] = (
        lines[offset + 2].split("!")[0].strip().lower() == "true"
    )
    nItem = int(lines[offset + 3].split("!")[0].strip())
    offset += 4
    secLength = yaml_data["lngth_wheel"]

    if nItem == 0:
        offset += secLength
    else:
        for itemNum in range(nItem):
            newDict = dict()
            newItem = {"Wheel": newDict}
            newDict["Index"] = itemNum
            for lineNum, line in enumerate(lines[offset + 0 : offset + secLength]):
                lineData = line.split("!")[0].strip()
                strData = lineData.split()
                match lineNum:
                    case 1:
                        newDict["Initial Momentum"] = float(lineData)
                    case 2:
                        newDict["Axis"] = [float(string) for string in strData]
                    case 3:
                        newDict["Max Torque"] = float(strData[0])
                        newDict["Max Momentum"] = float(strData[1])
                    case 4:
                        newDict["Rotor Inertia"] = float(lineData)
                    case 5:
                        newDict["Body"] = int(lineData)
                    case 6:
                        newDict["Node"] = int(lineData)
                    case 7:
                        newDict["Drag-Jitter File Name"] = lineData
            offset += secLength
            scDict["Wheels"].append(newItem)

    ### Magnetorquers
    nItem = int(lines[offset + 1].split("!")[0].strip())
    offset += 2
    secLength = yaml_data["lngth_mtb"]
    if nItem == 0:
        offset += secLength
    else:
        for itemNum in range(nItem):
            newDict = dict()
            newItem = {"MTB": newDict}
            newDict["Index"] = itemNum
            for lineNum, line in enumerate(lines[offset + 0 : offset + secLength]):
                lineData = line.split("!")[0].strip()
                strData = lineData.split()
                match lineNum:
                    case 1:
                        newDict["Saturation"] = float(lineData)
                    case 2:
                        newDict["Axis"] = [float(string) for string in strData]
                    case 3:
                        newDict["Node"] = int(lineData)
            offset += secLength
            scDict["MTBs"].append(newItem)

    ### Thrusters
    nItem = int(lines[offset + 1].split("!")[0].strip())
    offset += 2
    secLength = yaml_data["lngth_thrstrs"]
    if nItem == 0:
        offset += secLength
    else:
        for itemNum in range(nItem):
            newDict = dict()
            newItem = {"Thruster": newDict}
            newDict["Index"] = itemNum
            for lineNum, line in enumerate(lines[offset + 0 : offset + secLength]):
                lineData = line.split("!")[0].strip()
                strData = lineData.split()
                match lineNum:
                    case 1:
                        newDict["Mode"] = lineData
                    case 2:
                        newDict["Force"] = float(lineData)
                    case 3:
                        newDict["Axis"] = [float(string) for string in strData]
                    case 4:
                        newDict["Body"] = int(lineData)
                    case 5:
                        newDict["Node"] = int(lineData)
            offset += secLength
            scDict["Thrusters"].append(newItem)

    ### Gyros
    nItem = int(lines[offset + 1].split("!")[0].strip())
    offset += 2
    secLength = yaml_data["lngth_gyro"]
    if nItem == 0:
        offset += secLength
    else:
        for itemNum in range(nItem):
            newDict = dict()
            newItem = {"Gyro": newDict}
            newDict["Index"] = itemNum
            for lineNum, line in enumerate(lines[offset + 0 : offset + secLength]):
                lineData = line.split("!")[0].strip()
                strData = lineData.split()
                match lineNum:
                    case 1:
                        newDict["Sample Time"] = float(lineData)
                    case 2:
                        newDict["Axis"] = [float(string) for string in strData]
                    case 3:
                        newDict["Max Rate"] = float(lineData)
                    case 4:
                        newDict["Scale Factor"] = float(lineData)
                    case 5:
                        newDict["Quantization"] = float(lineData)
                    case 6:
                        newDict["Angle Random Walk"] = float(lineData)
                    case 7:
                        newDict["Bias Stability"] = float(strData[0])
                        newDict["Bias Stability Timespan"] = float(strData[1])
                    case 8:
                        newDict["Angle Noise"] = float(lineData)
                    case 9:
                        newDict["Initial Bias"] = float(lineData)
                    case 10:
                        newDict["Node"] = int(lineData)
            offset += secLength
            scDict["Gyros"].append(newItem)

    ### Magnetometers
    nItem = int(lines[offset + 1].split("!")[0].strip())
    offset += 2
    secLength = yaml_data["lngth_mag"]
    if nItem == 0:
        offset += secLength
    else:
        for itemNum in range(nItem):
            newDict = dict()
            newItem = {"Magnetometer": newDict}
            newDict["Index"] = itemNum
            for lineNum, line in enumerate(lines[offset + 0 : offset + secLength]):
                lineData = line.split("!")[0].strip()
                strData = lineData.split()
                match lineNum:
                    case 1:
                        newDict["Sample Time"] = float(lineData)
                    case 2:
                        newDict["Axis"] = [float(string) for string in strData]
                    case 3:
                        newDict["Saturation"] = float(lineData)
                    case 4:
                        newDict["Scale Factor"] = float(lineData)
                    case 5:
                        newDict["Quantization"] = float(lineData)
                    case 6:
                        newDict["Noise"] = float(lineData)
                    case 7:
                        newDict["Node"] = int(lineData)
            offset += secLength
            scDict["Magnetometers"].append(newItem)

    ### CSS
    nItem = int(lines[offset + 1].split("!")[0].strip())
    offset += 2
    secLength = yaml_data["lngth_css"]
    if nItem == 0:
        offset += secLength
    else:
        for itemNum in range(nItem):
            newDict = dict()
            newItem = {"CSS": newDict}
            newDict["Index"] = itemNum
            for lineNum, line in enumerate(lines[offset + 0 : offset + secLength]):
                lineData = line.split("!")[0].strip()
                strData = lineData.split()
                match lineNum:
                    case 1:
                        newDict["Sample Time"] = float(lineData)
                    case 2:
                        newDict["Axis"] = [float(string) for string in strData]
                    case 3:
                        newDict["Half Cone Angle"] = float(lineData)
                    case 4:
                        newDict["Scale Factor"] = float(lineData)
                    case 5:
                        newDict["Quantization"] = float(lineData)
                    case 6:
                        newDict["Body"] = int(lineData)
                    case 7:
                        newDict["Node"] = int(lineData)
            offset += secLength
            scDict["CSSs"].append(newItem)

    ### FSS
    nItem = int(lines[offset + 1].split("!")[0].strip())
    offset += 2
    secLength = yaml_data["lngth_fss"]
    if nItem == 0:
        offset += secLength
    else:
        for itemNum in range(nItem):
            newDict = dict()
            newItem = {"FSS": newDict}
            newDict["Index"] = itemNum
            for lineNum, line in enumerate(lines[offset + 0 : offset + secLength]):
                lineData = line.split("!")[0].strip()
                strData = lineData.split()
                match lineNum:
                    case 1:
                        newDict["Sample Time"] = float(lineData)
                    case 2:
                        newDict["Mounting Angles"] = dict()
                        newDict["Mounting Angles"]["Angles"] = [
                            float(string) for string in strData[0:3]
                        ]
                        newDict["Mounting Angles"]["Sequence"] = int(strData[3])
                    case 3:
                        newDict["Boresight Axis"] = lineData
                    case 4:
                        newDict["FOV Size"] = [float(string) for string in strData]
                    case 5:
                        newDict["Noise Equivalent Angle"] = float(lineData)
                    case 6:
                        newDict["Quantization"] = float(lineData)
                    case 7:
                        newDict["Node"] = int(lineData)
            offset += secLength
            scDict["FSSs"].append(newItem)

    ### Star Tracker
    nItem = int(lines[offset + 1].split("!")[0].strip())
    offset += 2
    secLength = yaml_data["lngth_strckr"]
    if nItem == 0:
        offset += secLength
    else:
        for itemNum in range(nItem):
            newDict = dict()
            newItem = {"ST": newDict}
            newDict["Index"] = itemNum
            for lineNum, line in enumerate(lines[offset + 0 : offset + secLength]):
                lineData = line.split("!")[0].strip()
                strData = lineData.split()
                match lineNum:
                    case 1:
                        newDict["Sample Time"] = float(lineData)
                    case 2:
                        newDict["Mounting Angles"] = dict()
                        newDict["Mounting Angles"]["Angles"] = [
                            float(string) for string in strData[0:3]
                        ]
                        newDict["Mounting Angles"]["Sequence"] = int(strData[3])
                    case 3:
                        newDict["Boresight Axis"] = lineData
                    case 4:
                        newDict["FOV Size"] = [float(string) for string in strData]
                    case 5:
                        newDict["Exclusion Angles"] = dict()
                        newDict["Exclusion Angles"]["Sun"] = float(strData[0])
                        newDict["Exclusion Angles"]["Earth"] = float(strData[1])
                        newDict["Exclusion Angles"]["Luna"] = float(strData[2])
                    case 6:
                        newDict["Noise Equivalent Angle"] = [
                            float(string) for string in strData
                        ]
                    case 7:
                        newDict["Node"] = int(lineData)
            offset += secLength
            scDict["STs"].append(newItem)

    ### GPS
    nItem = int(lines[offset + 1].split("!")[0].strip())
    offset += 2
    secLength = yaml_data["lngth_gps"]
    if nItem == 0:
        offset += secLength
    else:
        for itemNum in range(nItem):
            newDict = dict()
            newItem = {"GPS": newDict}
            newDict["Index"] = itemNum
            for lineNum, line in enumerate(lines[offset + 0 : offset + secLength]):
                lineData = line.split("!")[0].strip()
                strData = lineData.split()
                match lineNum:
                    case 1:
                        newDict["Sample Time"] = float(lineData)
                    case 2:
                        newDict["Position Noise"] = float(lineData)
                    case 3:
                        newDict["Velocity Noise"] = float(lineData)
                    case 4:
                        newDict["Time Noise"] = float(lineData)
                    case 5:
                        newDict["Node"] = int(lineData)
            offset += secLength
            scDict["GPSs"].append(newItem)

    ### Accelerometer
    nItem = int(lines[offset + 1].split("!")[0].strip())
    offset += 2
    secLength = yaml_data["lngth_accel"]
    if nItem == 0:
        offset += secLength
    else:
        for itemNum in range(nItem):
            newDict = dict()
            newItem = {"Accelerometer": newDict}
            newDict["Index"] = itemNum
            for lineNum, line in enumerate(lines[offset + 0 : offset + secLength]):
                lineData = line.split("!")[0].strip()
                strData = lineData.split()
                match lineNum:
                    case 1:
                        newDict["Sample Time"] = float(lineData)
                    case 2:
                        newDict["Axis"] = [float(string) for string in strData]
                    case 3:
                        newDict["Max Acceleration"] = float(lineData)
                    case 4:
                        newDict["Scale Factor"] = float(lineData)
                    case 5:
                        newDict["Quantization"] = float(lineData)
                    case 6:
                        newDict["DV Random Walk"] = float(lineData)
                    case 7:
                        newDict["Bias Stability"] = float(strData[0])
                        newDict["Bias Stability Timespan"] = float(strData[1])
                    case 8:
                        newDict["DV Noise"] = float(lineData)
                    case 9:
                        newDict["Initial Bias"] = float(lineData)
                    case 10:
                        newDict["Node"] = int(lineData)
            offset += secLength
            scDict["Accelerometers"].append(newItem)

    scDict = convertToYamlAndComment(scDict, commentDict)

    for body in scDict["Bodies"]:
        body["Body"].yaml_set_anchor("Body_%0d" % body["Body"]["Index"])

    for wheel in scDict["Wheels"]:
        iB = wheel["Wheel"]["Body"]
        for Body in scDict["Bodies"]:
            if Body["Body"]["Index"] == iB:
                wheel["Wheel"]["Body"] = Body["Body"]
                break
    for css in scDict["CSSs"]:
        iB = css["CSS"]["Body"]
        for Body in scDict["Bodies"]:
            if Body["Body"]["Index"] == iB:
                css["CSS"]["Body"] = Body["Body"]
                break
    for thr in scDict["Thrusters"]:
        iB = thr["Thruster"]["Body"]
        for Body in scDict["Bodies"]:
            if Body["Body"]["Index"] == iB:
                thr["Thruster"]["Body"] = Body["Body"]
                break

    scDict["Wheel Params"].fa.set_flow_style()

    return scDict


# %%
def convertOrb(missionDir, yaml, fileName, commentDict=None):
    orbName = fileName[4:-4]

    orbDict = dict()
    confDict = orbDict["Configuration"] = dict()
    orbitDict = orbDict["Orbit"] = dict()
    confDict["Name"] = orbName

    with open(missionDir + "/InOut/" + fileName, "r") as f:
        lines = f.readlines()

    for lineNum, line in enumerate(lines[0:3]):
        lineData = line.split("!")[0].strip()
        strData = lineData.split()
        match (lineNum):
            case 1:
                confDict["Description"] = lineData.strip('"')
            case 2:
                orbitDict["Type"] = lineData

    match (orbitDict["Type"].lower()):
        case "zero":
            for lineNum, line in enumerate(lines[3:6]):
                lineData = line.split("!")[0].strip()
                match (lineNum):
                    case 1:
                        orbitDict["World"] = lineData
                    case 2:
                        orbitDict["Polyhedron Grav"] = lineData.lower() == "true"
        case "flight":
            for lineNum, line in enumerate(lines[6:9]):
                lineData = line.split("!")[0].strip()
                match (lineNum):
                    case 1:
                        orbitDict["Region"] = int(lineData)
                    case 2:
                        orbitDict["Polyhedron Grav"] = lineData.lower() == "true"
        case "central":
            for lineNum, line in enumerate(lines[9:13]):
                lineData = line.split("!")[0].strip()
                match (lineNum):
                    case 1:
                        orbitDict["World"] = lineData
                    case 2:
                        orbitDict["J2 Secular Drift"] = lineData.lower() == "true"
                    case 3:
                        initDict = orbitDict["Init"] = dict()
                        initDict["Method"] = lineData
            match initDict["Method"].lower():
                case "kep":
                    kepType = lines[13].split("!")[0].strip()
                    initDict["SMA Parameterization"] = kepType
                    match kepType.lower():
                        case "pa":
                            data = lines[14].split("!")[0].strip()
                            initDict["Periapsis"] = float(data.split()[0])
                            initDict["Apoapsis"] = float(data.split()[1])
                        case "ae":
                            data = lines[15].split("!")[0].strip()
                            initDict["Minimum Altitude"] = float(data.split()[0])
                            initDict["Eccentricity"] = float(data.split()[1])
                    initDict["Inclination"] = float(lines[16].split("!")[0].strip())
                    initDict["RAAN"] = float(lines[17].split("!")[0].strip())
                    initDict["Arg of Periapsis"] = float(
                        lines[18].split("!")[0].strip()
                    )
                    initDict["True Anomaly"] = float(lines[19].split("!")[0].strip())
                case "rv":
                    data = lines[20].split("!")[0].strip().split()
                    initDict["Position"] = [float(string) for string in data]
                    data = lines[21].split("!")[0].strip().split()
                    initDict["Velocity"] = [float(string) for string in data]
                case "file":
                    initDict["File Type"] = lines[22].split("!")[0].strip()
                    initDict["File Name"] = lines[23].split("!")[0].strip(' "')
                    initDict["Label in File"] = lines[24].split("!")[0].strip(' "')
        case "three_body":
            for lineNum, line in enumerate(lines[25:29]):
                lineData = line.split("!")[0].strip()
                match (lineNum):
                    case 1:
                        orbitDict["Lagrange System"] = lineData
                    case 2:
                        orbitDict["Propagation Method"] = lineData
                    case 3:
                        initDict = orbitDict["Init"] = dict()
                        initDict["Method"] = lineData
            match initDict["Method"].lower():
                case "modes":
                    for lineNum, line in enumerate(lines[29:33] + lines[36:38]):
                        lineData = line.split("!")[0].strip()
                        match (lineNum):
                            case 0:
                                initDict["Lagrange Point"] = lineData
                            case 1:
                                initDict["XY SMA"] = float(lineData)
                            case 2:
                                initDict["XY Phase"] = float(lineData)
                            case 3:
                                initDict["Sense"] = lineData
                            case 4:
                                initDict["Z SMA"] = float(lineData)
                            case 8:
                                initDict["Z Phase"] = float(lineData)
                    if (
                        initDict["Lagrange Point"] == "L4"
                        or initDict["Lagrange Point"] == "L5"
                    ):
                        for lineNum, line in enumerate(lines[33:36]):
                            lineData = line.split("!")[0].strip()
                            match lineNum:
                                case 0:
                                    initDict["XY 2nd SMA"] = float(lineData)
                                case 1:
                                    initDict["XY 2nd Phase"] = float(lineData)
                                case 2:
                                    initDict["2nd Sense"] = lineData
                case "xyz":
                    for lineNum, line in enumerate(lines[38:40]):
                        lineData = line.split("!")[0].strip()
                        strData = lineData.split()
                        match lineNum:
                            case 1:
                                initDict["Position"] = [
                                    float(string) for string in strData
                                ]
                            case 2:
                                initDict["Velocity"] = [
                                    float(string) for string in strData
                                ]
                case "file":
                    for lineNum, line in enumerate(lines[40:42]):
                        lineData = line.split("!")[0].strip()
                        strData = lineData.split()
                        match lineNum:
                            case 1:
                                initDict["File Type"] = strData[0]
                                initDict["File Name"] = strData[1]('"')
                            case 2:
                                initDict["Label in File"] = lineData.strip('"')

    formDict = orbDict["Formation"] = dict()
    for lineNum, line in enumerate(lines[42:47]):
        lineData = line.split("!")[0].strip()
        strData = lineData.split()
        match (lineNum):
            case 1:
                formDict["Fixed Frame"] = lineData
            case 2:
                formDict["Euler Angles"] = dict()
                formDict["Euler Angles"]["Angles"] = [
                    float(string) for string in strData[0:3]
                ]
                formDict["Euler Angles"]["Sequence"] = int(strData[3])
            case 3:
                formDict["Expression Frame"] = lineData
            case 4:
                formDict["Position"] = [float(string) for string in strData]

    orbDict = convertToYamlAndComment(orbDict, commentDict)
    return orbDict


# %%
def convertNodes(missionDir, yaml, fileName, commentDict=None):
    nodeDict = dict()
    confDict = nodeDict["Configuration"] = dict()
    confDict["Name"] = fileName

    with open(missionDir + "/InOut/" + fileName, "r") as f:
        lines = f.readlines()

    confDict["Description"] = lines[1].split("!")[0].strip()
    nNodes = int(lines[2].split("!")[0].strip())
    nodeDict["Nodes"] = list()
    for nodeNum, line in enumerate(lines[4 : 4 + nNodes]):
        lineData = line.split('"')
        location = lineData[0].strip().split()
        newDict = dict()
        newItem = {"Node": newDict}
        newDict["Index"] = nodeNum
        newDict["Location"] = [float(string) for string in location]
        newDict["Comment"] = lineData[1].strip()
        nodeDict["Nodes"].append(newItem)

    nodeDict = convertToYamlAndComment(nodeDict, commentDict)

    return nodeDict


# %%
def convertFOV(missionDir, yaml, fovFileName="Inp_FOV.txt", commentDict=None):
    fovDict = dict()
    fovDict["FOVs"] = fovs = list()
    with open(missionDir + "/InOut/" + fovFileName, "r") as f:
        lines = f.readlines()
    nFOV = int(lines[1].split("!")[0].strip())
    if nFOV > 0:
        offset = 2
        secLength = 11
        for _ in range(nFOV):
            newDict = dict()
            newItem = {"FOV": newDict}
            for lineNum, line in enumerate(lines[offset : offset + secLength]):
                lineData = line.split("!")[0].strip()
                strData = lineData.split()
                match lineNum:
                    case 1:
                        newDict["Label"] = lineData.strip('"')
                    case 2:
                        newDict["Sides"] = dict()
                        newDict["Sides"]["Number"] = int(strData[0])
                        newDict["Sides"]["Length"] = float(strData[1])
                    case 3:
                        newDict["Width"] = float(strData[0])
                        newDict["Height"] = float(strData[1])
                    case 4:
                        newDict["Color"] = dict()
                        newDict["Color"]["RGB"] = [
                            float(string) for string in strData[0:3]
                        ]
                        newDict["Color"]["Alpha"] = float(strData[3])
                    case 5:
                        newDict["Type"] = lineData
                    case 6:
                        newDict["Near Field"] = strData[0].lower() == "true"
                        newDict["Far Field"] = strData[1].lower() == "true"
                    case 7:
                        newDict["SC"] = int(strData[0])
                        newDict["Body"] = int(strData[1])
                    case 8:
                        newDict["Position"] = [float(string) for string in strData]
                    case 9:
                        newDict["Euler Angles"] = dict()
                        newDict["Euler Angles"]["Angles"] = [
                            float(string) for string in strData[0:3]
                        ]
                        newDict["Euler Angles"]["Sequence"] = int(strData[3])
                    case 10:
                        newDict["Boresight"] = lineData
            offset += secLength
            fovs.append(newItem)

    fovDict = convertToYamlAndComment(fovDict, commentDict)

    return fovDict


# %%
def convertGraphics(
    missionDir, yaml, graphicsFileName="Inp_Graphics.txt", commentDict=None
):
    graphicsDict = dict()
    graphicsDict["Configuration"] = configDict = dict()
    configDict["Name"] = graphicsFileName

    with open(missionDir + "/InOut/" + graphicsFileName, "r") as f:
        lines = f.readlines()

    for lineNum, line in enumerate(lines[0:24]):
        lineData = line.split("!")[0].strip()
        strData = lineData.split()
        match lineNum:
            case 1:
                configDict["Output Interval"] = float(lineData)
            case 2:
                configDict["Star Catalog File"] = lineData
            case 3:
                configDict["Map Exists"] = lineData.lower() == "true"
            case 4:
                configDict["Orrery Exists"] = lineData.lower() == "true"
            case 5:
                configDict["Unit Sphere Exists"] = lineData.lower() == "true"
            case 7:
                configDict["Pause on Startup"] = lineData.lower() == "true"
            case 8:
                povDict = graphicsDict["POV"] = dict()
                povDict["Mode"] = lineData
            case 9:
                hostDict = povDict["Host"] = dict()
                hostDict["Type"] = lineData
            case 10:
                hostDict["SC"] = int(strData[0])
                hostDict["Body"] = int(strData[1])
                hostDict["Frame"] = strData[2]  # Does nothing
            case 11:
                TargetDict = povDict["Target"] = dict()
                TargetDict["Type"] = lineData
            case 12:
                TargetDict["SC"] = int(strData[0])
                TargetDict["Body"] = int(strData[1])
                TargetDict["Frame"] = strData[2]
            case 13:
                povDict["Boresight Axis"] = lineData
            case 14:
                povDict["Up Axis"] = lineData
            case 15:
                povDict["POV Range"] = float(lineData)
            case 16:
                povDict["POV Vertical Angle"] = float(lineData)
            case 17:
                povDict["POV Host Position"] = [float(string) for string in strData]
            case 18:
                povDict["POV View"] = lineData
            case 20:
                camDict = graphicsDict["Cam"] = dict()
                camDict["Title"] = lineData.strip('"')
            case 21:
                dimDict = camDict["Dimensions"] = dict()
                dimDict["Width"] = int(strData[0])
                dimDict["Height"] = int(strData[1])
            case 22:
                camDict["Mouse Scale Factor"] = float(lineData)
            case 23:
                camDict["Gamma Exponent"] = float(lineData)
    showLabels = [
        "N Axes",
        "L Axes",
        "F Axes",
        "B Axes",
        "N Grid",
        "L Grid",
        "F Grid",
        "B Grid",
        "G Grid",
        "Fields of View",
        "Prox Ops",
        "TDRS Satellites",
        "Shadows",
        "Astro Labels",
        "Truth Vectors",
        "FSW Vectors",
        "Milky Way",
        "Fermi Sky",
    ]
    showDict = camDict["Cam Show"] = dict()
    for lineNum, line in enumerate(lines[25:43]):
        lineData = line.split("!")[0].strip()
        strData = lineData.split()
        showDict[showLabels[lineNum]] = dict()
        showDict[showLabels[lineNum]]["Show"] = strData[0].lower() == "true"
        showDict[showLabels[lineNum]]["Label"] = " ".join(strData[1:]).strip('"')

    mapDict = graphicsDict["Map"] = dict()
    for lineNum, line in enumerate(lines[43:46]):
        lineData = line.split("!")[0].strip()
        strData = lineData.split()
        match lineNum:
            case 1:
                mapDict["Title"] = lineData.strip('"')
            case 2:
                dimDict = mapDict["Dimensions"] = dict()
                dimDict["Width"] = int(strData[0])
                dimDict["Height"] = int(strData[1])

    showLabels = ["Clock", "Tlm Clock", "Credits", "Night"]
    showDict = mapDict["Map Show"] = dict()
    for lineNum, line in enumerate(lines[47:51]):
        lineData = line.split("!")[0].strip()
        strData = lineData.split()
        showDict[showLabels[lineNum]] = dict()
        showDict[showLabels[lineNum]]["Show"] = strData[0].lower() == "true"
        showDict[showLabels[lineNum]]["Label"] = " ".join(strData[1:]).strip('"')

    showLabels = ["Major", "Zodiac", "Minor"]
    showDict = graphicsDict["Constellations Show"] = dict()
    for lineNum, line in enumerate(lines[52:55]):
        lineData = line.split("!")[0].strip()
        strData = lineData.split()
        showDict[showLabels[lineNum]] = dict()
        showDict[showLabels[lineNum]]["Show"] = strData[0].lower() == "true"

    graphicsDict = convertToYamlAndComment(graphicsDict, commentDict)

    return graphicsDict


# %%
def convertIPC(missionDir, yaml, ipcFileName="Inp_IPC.txt", commentDict=None):
    ipcDict = dict()
    ipcDict["IPCs"] = ipcs = list()
    with open(missionDir + "/InOut/" + ipcFileName, "r") as f:
        lines = f.readlines()

    nIPC = int(lines[1].split("!")[0].strip())
    if nIPC > 0:
        offset = 2
        secLength = 9
        for _ in range(nIPC):
            newDict = dict()
            newItem = {"IPC": newDict}
            for lineNum, line in enumerate(lines[offset : offset + secLength]):
                lineData = line.split("!")[0].strip()
                strData = lineData.split()
                match lineNum:
                    case 1:
                        newDict["Mode"] = lineData
                    case 2:
                        newDict["AC ID"] = int(lineData)
                    case 3:
                        newDict["File Name"] = lineData.strip('"')
                    case 4:
                        socketDict = newDict["Socket"] = dict()
                        socketDict["Role"] = lineData
                    case 5:
                        hostDict = socketDict["Host"] = dict()
                        hostDict["Name"] = strData[0]
                        hostDict["Port"] = int(strData[1])
                    case 6:
                        socketDict["Blocking"] = lineData.lower() == "true"
                    case 7:
                        newDict["Echo to stdout"] = lineData.lower() == "true"
                    case 8:
                        nPrefixes = int(lineData)
                        if nPrefixes > 0:
                            newDict["Prefixes"] = list()
                            for i in range(nPrefixes):
                                lineData = (
                                    lines[offset + lineNum + i + 1]
                                    .split("!")[0]
                                    .strip(' "')
                                )
                                newDict["Prefixes"].append(lineData)
            offset += secLength + nPrefixes
            ipcs.append(newItem)

    ipcDict = convertToYamlAndComment(ipcDict, commentDict)

    return ipcDict


# %%
def convertRegion(missionDir, yaml, regionFileName="Inp_Region.txt", commentDict=None):
    regDict = dict()
    regDict["Regions"] = regions = list()
    with open(missionDir + "/InOut/" + regionFileName, "r") as f:
        lines = f.readlines()

    nRegions = int(lines[1].split("!")[0].strip())
    if nRegions > 0:
        offset = 2
        secLength = 9
        for _ in range(nRegions):
            newDict = dict()
            newItem = {"Region": newDict}
            for lineNum, line in enumerate(lines[offset : offset + secLength]):
                lineData = line.split("!")[0].strip()
                strData = lineData.split()
                match lineNum:
                    case 1:
                        newDict["Exists"] = lineData.lower() == "true"
                    case 2:
                        newDict["Name"] = lineData.strip('"')
                    case 3:
                        newDict["World"] = lineData
                    case 4:
                        locDict = newDict["Location"] = dict()
                        locDict["Type"] = lineData
                    case 5:
                        if locDict["Type"].lower() == "posw":
                            locDict["Position"] = [
                                float(string) for string in strData[0:3]
                            ]
                    case 6:
                        if locDict["Type"].lower() == "lla":
                            locDict["Position"] = [
                                float(string) for string in strData[0:3]
                            ]
                    case 7:
                        coefDict = newDict["Coefficients"] = dict()
                        coefDict["Elasticity"] = float(strData[0])
                        coefDict["Damping"] = float(strData[1])
                        coefDict["Friction"] = float(strData[2])
                    case 8:
                        newDict["Geometry File Name"] = lineData.strip('"')
            offset += secLength
            regions.append(newItem)

    regDict = convertToYamlAndComment(regDict, commentDict)

    return regDict


# %%
def convertTDRS(missionDir, yaml, tdrsFileName="Inp_TDRS.txt", commentDict=None):
    tdrs = dict()
    tdrsList = tdrs["TDRSs"] = list()
    with open(missionDir + "/InOut/" + tdrsFileName, "r") as f:
        lines = f.readlines()

    nTDRS = 10
    for lineNum, line in enumerate(lines[1 : 1 + nTDRS]):
        lineData = line.split("!")[0].strip()
        strData = lineData.split()
        newDict = dict()
        newItem = {"TDRS": newDict}
        newDict["Number"] = lineNum + 1
        newDict["Exists"] = strData[0].lower() == "true"
        newDict["Label"] = " ".join(strData[1:]).strip('"')
        tdrsList.append(newItem)

    tdrs = convertToYamlAndComment(tdrs, commentDict)

    return tdrs


# %%
def convertNOS3(missionDir, yaml, nos3FileName="Inp_NOS3.txt", commentDict=None):
    outDict = dict()
    nos3 = outDict["Configuration"] = dict()
    with open(missionDir + "/InOut/" + nos3FileName, "r") as f:
        lines = f.readlines()

    nos3["Bus"] = lines[1].split("!")[0].strip()
    nos3["Connection String"] = lines[2].split("!")[0].strip()

    outDict = convertToYamlAndComment(outDict, commentDict)

    return outDict


# %%
def nukeIndex(dsmMap):
    if isinstance(dsmMap, ruamel.yaml.comments.CommentedMap):
        if "Index" in dsmMap:
            del dsmMap["Index"]
            return
        else:
            for key in dsmMap.keys():
                nukeIndex(dsmMap[key])
    elif isinstance(dsmMap, ruamel.yaml.comments.CommentedSeq):
        for item in dsmMap:
            nukeIndex(item)


def convertDSM(missionDir, yaml, dsmFileName="Inp_DSM.txt", commentDict=None):
    dsm = ruamel.yaml.comments.CommentedMap()
    dsmTypes = [
        "Gains",
        "Limits",
        "Actuator",
        "Controller",
        "Position",
        "One Vector",
        "Secondary Vector",
        "Quaternion",
        "Mirror",
        "Detumble",
        "Whl H Manage",
        "Actuator Cmd",
        "Maneuver",
        "Sensor",
        "Sensor Set",
        "Navigation Data",
        "Navigation",
        "DSM Commands",
    ]

    for typeName in dsmTypes:
        if not typeName.startswith("DSM Commands"):
            dsm[typeName + " Configurations"] = list()
        else:
            dsm["Two Vector Configurations"] = list()
            dsm[typeName] = list()

    with open(missionDir + "/InOut/" + dsmFileName, "r") as f:
        while True:
            line = f.readline()
            if not line or bool(re.match("eof", line, re.I)):
                break
            lineData = line.split("#")[0].strip()
            strData = lineData.split()
            if line.startswith("DSM_Cmd"):
                scIndex = int(re.search(r"\d+", strData[1]).group())
                cmdTime = float(strData[3])
                foundDict = False
                for scDict in dsm["DSM Commands"]:
                    if scIndex == scDict["SC"]:
                        foundDict = True
                        break
                if not foundDict:
                    scDict = dict()
                    dsm["DSM Commands"].append(scDict)
                    scDict["SC"] = scIndex
                    scDict["Command Sequence"] = list()

                newDict = dict()
                newDict["Time"] = cmdTime
                newDict["Commands"] = list()
                for command in strData[5:]:
                    cmdDict = dict()
                    search = re.search(r"\d+", command)
                    if search:
                        index = int(search.group())
                    if command.startswith("TranslationCmd_"):
                        cmdDict["Type"] = "Translation"
                        cmdDict["Subtype"] = "Position"
                        cmdDict["Index"] = index
                    elif command.startswith("AttitudeCmd_PV"):
                        cmdDict["Type"] = "Attitude"
                        if "_SV" in command:
                            indicies = re.findall(r"\d+", command)
                            cmdDict["Subtype"] = "Two Vector"
                            cmdDict["Index"] = [int(ind) for ind in indicies]
                        else:
                            cmdDict["Subtype"] = "One Vector"
                            cmdDict["Index"] = index
                    elif command.startswith("QuaternionCmd_"):
                        cmdDict["Type"] = "Attitude"
                        cmdDict["Subtype"] = "Quaternion"
                        cmdDict["Index"] = index
                    elif command.startswith("MirrorCmd_"):
                        cmdDict["Type"] = "Attitude"
                        cmdDict["Subtype"] = "Mirror"
                        cmdDict["Index"] = index
                    elif command.startswith("DetumbleCmd_"):
                        cmdDict["Type"] = "Attitude"
                        cmdDict["Subtype"] = "Detumble"
                        cmdDict["Index"] = index
                    elif command.startswith("WhlHManageCmd_"):
                        cmdDict["Type"] = "Attitude"
                        cmdDict["Subtype"] = "Whl H Manage"
                        cmdDict["Index"] = index
                    elif command.startswith("ActuatorCmd_"):
                        cmdDict["Type"] = "Actuator"
                        cmdDict["Index"] = index
                    elif command.startswith("ManeuverCmd_"):
                        cmdDict["Type"] = "Translation"
                        cmdDict["Subtype"] = "Maneuver"
                        cmdDict["Index"] = index
                    elif command.startswith("NavigationCmd_"):
                        cmdDict["Type"] = "Navigation"
                        cmdDict["Index"] = index
                    elif "passive" in command.lower():
                        if "trn" in command.lower():
                            cmdDict["Type"] = "Attitude"
                            cmdDict["Subtype"] = "Passive"
                        elif "att" in command.lower():
                            cmdDict["Type"] = "Translation"
                            cmdDict["Subtype"] = "Passive"
                        elif "nav" in command.lower():
                            cmdDict["Type"] = "Navigation"
                            cmdDict["Subtype"] = "Passive"
                    newDict["Commands"].append(cmdDict)

                scDict["Command Sequence"].append(newDict)
            elif not (
                line.startswith("#")
                or line.isspace()
                or line.startswith("<")
                or bool(re.match("end_of_file", line, re.I))
                or bool(re.match("eof", line, re.I))
            ):
                if line.startswith("TranslationCmd_"):
                    typeName = "Position"
                elif line.startswith("AttitudeCmd_PV"):
                    typeName = "One Vector"
                elif line.startswith("AttitudeCmd_SV"):
                    typeName = "Secondary Vector"
                elif line.startswith("QuaternionCmd_"):
                    typeName = "Quaternion"
                elif line.startswith("MirrorCmd_"):
                    typeName = "Mirror"
                elif line.startswith("DetumbleCmd_"):
                    typeName = "Detumble"
                elif line.startswith("WhlHManageCmd_"):
                    typeName = "Whl H Manage"
                elif line.startswith("ActuatorCmd_"):
                    typeName = "Actuator Cmd"
                elif line.startswith("ManeuverCmd_"):
                    typeName = "Maneuver"
                elif line.startswith("Controller_"):
                    typeName = "Controller"
                elif line.startswith("Actuators_"):
                    typeName = "Actuator"
                elif line.startswith("Gains_"):
                    typeName = "Gains"
                elif line.startswith("Limits_"):
                    typeName = "Limits"
                elif line.startswith("NavigationCmd_"):
                    typeName = "Navigation"
                elif line.startswith("Dat_"):
                    typeName = "Navigation Data"
                elif line.startswith("SensorSet_"):
                    typeName = "Sensor Set"
                elif line.startswith("Sensor_"):
                    typeName = "Sensor"

                newItem = dict()
                newDict = newItem[typeName] = dict()
                newDict["Index"] = int(re.search(r"\d+", strData[0]).group())
                if "#" in line:
                    newDict["Description"] = line.split("#")[1].strip()
                else:
                    newDict["Description"] = strData[0]
                if line.startswith("TranslationCmd_"):
                    newDict["Position"] = [float(string) for string in strData[1:4]]
                    newDict["Origin"] = strData[4]
                    newDict["Frame"] = strData[5]
                    newDict["Controller"] = int(re.search(r"\d+", strData[6]).group())
                    newDict["Actuator"] = int(re.search(r"\d+", strData[7]).group())
                elif line.startswith("AttitudeCmd_PV"):
                    pvDict = newDict["Primary Vector"] = dict()
                    tgtDict = pvDict["Target"] = dict()
                    tgtDict["Type"] = strData[1]
                    pvDict["Axis"] = [float(string) for string in strData[2:5]]
                    if tgtDict["Type"].lower() == "vec":
                        tgtDict["Frame"] = strData[5]
                        tgtDict["Axis"] = [float(string) for string in strData[6:9]]
                        newDict["Controller"] = int(
                            re.search(r"\d+", strData[9]).group()
                        )
                        newDict["Actuator"] = int(
                            re.search(r"\d+", strData[10]).group()
                        )
                    else:
                        tgtDict["Target"] = strData[5]
                        newDict["Controller"] = int(
                            re.search(r"\d+", strData[6]).group()
                        )
                        newDict["Actuator"] = int(re.search(r"\d+", strData[7]).group())
                elif line.startswith("AttitudeCmd_SV"):
                    tgtDict = newDict["Target"] = dict()
                    tgtDict["Type"] = strData[1]
                    newDict["Axis"] = [float(string) for string in strData[2:5]]
                    if tgtDict["Type"].lower() == "vec":
                        tgtDict["Frame"] = strData[5]
                        tgtDict["Axis"] = [float(string) for string in strData[6:9]]
                    else:
                        tgtDict["Target"] = strData[5]
                elif line.startswith("QuaternionCmd_"):
                    newDict["Quaternion"] = [float(string) for string in strData[1:5]]
                    newDict["Frame"] = strData[5]
                    newDict["Controller"] = int(re.search(r"\d+", strData[6]).group())
                    newDict["Actuator"] = int(re.search(r"\d+", strData[7]).group())
                elif line.startswith("MirrorCmd_"):
                    newDict["Target"] = strData[1]
                    newDict["Controller"] = int(re.search(r"\d+", strData[2]).group())
                    newDict["Actuator"] = int(re.search(r"\d+", strData[3]).group())
                elif line.startswith("DetumbleCmd_"):
                    newDict["Controller"] = int(re.search(r"\d+", strData[1]).group())
                    newDict["Actuator"] = int(re.search(r"\d+", strData[2]).group())
                elif line.startswith("WhlHManageCmd_"):
                    newDict["Dumping"] = strData[1].lower() == "on"
                    newDict["Minimum H_norm"] = float(strData[2])
                    newDict["Maximum H_norm"] = float(strData[3])
                    newDict["Controller"] = int(re.search(r"\d+", strData[4]).group())
                    newDict["Actuator"] = int(re.search(r"\d+", strData[5]).group())
                elif line.startswith("ActuatorCmd_"):
                    newDict["Actuators"] = list()
                    if len(strData) >= 3:
                        for act in strData[2:]:
                            actDict = dict()
                            data = parse("{:w}_[{:d}]_[{:f}]", act)
                            if data is not None:
                                dataTuple = data.fixed
                                actDict["Type"] = dataTuple[0]
                                actDict["Index"] = dataTuple[1]
                                actDict["Duty Cycle"] = dataTuple[2]
                                newDict["Actuators"].append(actDict)
                elif line.startswith("ManeuverCmd_"):
                    newDict["Delta V"] = [float(string) for string in strData[1:4]]
                    newDict["Frame"] = strData[4]
                    newDict["Type"] = strData[5]
                    newDict["Duration"] = float(strData[6])
                    newDict["Limits"] = int(re.search(r"\d+", strData[7]).group())
                    newDict["Actuator"] = int(re.search(r"\d+", strData[8]).group())
                elif line.startswith("Controller_"):
                    newDict["Type"] = strData[1]
                    newDict["Gains"] = int(re.search(r"\d+", strData[2]).group())
                    newDict["Limits"] = int(re.search(r"\d+", strData[3]).group())
                elif line.startswith("Actuators_"):
                    newDict["Index"] = int(re.search(r"\d+", strData[0]).group())
                    newDict["Type"] = strData[1]
                    if "#" in line:
                        newDict["Description"] = line.split("#")[1].strip()
                    else:
                        newDict["Description"] = strData[0]
                elif line.startswith("Gains_"):
                    newDict["Type"] = strData[1]
                    gainDict = newDict["Gains"] = dict()
                    match newDict["Type"].lower():
                        case "pid":
                            for i in range(2, 15, 4):
                                type = strData[i]
                                match type.lower():
                                    case "kp":
                                        gainDict["Kp"] = [
                                            float(string)
                                            for string in strData[i + 1 : i + 4]
                                        ]
                                    case "kr":
                                        gainDict["Kr"] = [
                                            float(string)
                                            for string in strData[i + 1 : i + 4]
                                        ]
                                    case "ki":
                                        gainDict["Ki"] = [
                                            float(string)
                                            for string in strData[i + 1 : i + 4]
                                        ]
                                    case "ki_limit":
                                        gainDict["Ki_Limit"] = [
                                            float(string)
                                            for string in strData[i + 1 : i + 4]
                                        ]
                        case "pid_wn":
                            gainDict["Omega"] = float(strData[2])
                            gainDict["Zeta"] = float(strData[3])
                            gainDict["Alpha"] = float(strData[4])
                            gainDict["Ki_Limit"] = float(strData[5])
                        case "momentumdump":
                            gainDict["Kp"] = [float(string) for string in strData[2:5]]
                        case "fc_lya":
                            gainDict["K_lya"] = [
                                float(string) for string in strData[2:]
                            ]
                        case "custom":
                            gainDict["K"] = [float(string) for string in strData[2:]]
                elif line.startswith("Limits_"):
                    newDict["Force Max"] = [float(string) for string in strData[1:4]]
                    newDict["Velocity Max"] = [float(string) for string in strData[4:7]]
                elif line.startswith("NavigationCmd_"):
                    newDict["Type"] = strData[1]
                    newDict["Batching"] = strData[2]
                    newDict["Frame"] = strData[3]
                    newDict["Reference Origin"] = strData[4]
                    dataDict = newDict["Data"] = dict()
                    dataDict["Q"] = int(re.search(r"\d+", strData[5]).group())
                    dataDict["P"] = int(re.search(r"\d+", strData[6]).group())
                    dataDict["x0"] = int(re.search(r"\d+", strData[7]).group())
                    newDict["Sensor Set"] = int(re.search(r"\d+", strData[8]).group())
                    newDict["States"] = strData[9:]
                elif line.startswith("Dat_"):
                    for i, str in enumerate(strData[1:]):
                        if str.lower() == "pos":
                            newDict["Position"] = [
                                float(item) for item in strData[i + 2 : i + 5]
                            ]
                        elif str.lower() == "vel":
                            newDict["Velocity"] = [
                                float(item) for item in strData[i + 2 : i + 5]
                            ]
                        elif str.lower() == "omega":
                            newDict["Omega"] = [
                                float(item) for item in strData[i + 2 : i + 5]
                            ]
                        elif str.lower() == "attitude":
                            if strData[i + 5] and strData[i + 5].isnumeric():
                                newDict["Attitude"] = dict()
                                newDict["Attitude"]["Angles"] = [
                                    float(item) for item in strData[i + 2 : i + 5]
                                ]
                                newDict["Attitude"]["Sequence"] = int(strData[i + 5])
                            else:
                                newDict["Attitude"] = [
                                    float(item) for item in strData[i + 2 : i + 5]
                                ]
                elif line.startswith("SensorSet_"):
                    newDict["Sensors"] = [
                        int(re.search(r"\d+", item).group()) for item in strData[1:]
                    ]
                elif line.startswith("Sensor_"):
                    newDict["Type"] = strData[1]
                    newDict["Sensor Index"] = int(strData[2])
                    newDict["Sensor Noise"] = [float(item) for item in strData[3:-2]]
                    newDict["Underweighting Factor"] = float(strData[-2])
                    newDict["Residual Editing Gate"] = float(strData[-1])

                dsm[typeName + " Configurations"].append(newItem)

    dsm = convertToYamlAndComment(dsm, commentDict)
    for scDict in dsm["DSM Commands"]:
        for cmdTime in scDict["Command Sequence"]:
            for cmd in cmdTime["Commands"]:
                if cmd["Type"] == "Actuator":
                    subType = "Actuator Cmd"
                else:
                    subType = cmd["Subtype"]
                if "Two Vector" in subType:
                    inds = cmd["Index"]
                    haveDict = False
                    for config in dsm["Two Vector Configurations"]:
                        twoVec = config["Two Vector"]
                        if twoVec["Index"] == inds:
                            haveDict = True
                            cmd["Command Data"] = config["Two Vector"]
                            break
                    if not haveDict:
                        headDict = ruamel.yaml.comments.CommentedMap()
                        headDict["Two Vector"] = twoVecCmd = (
                            ruamel.yaml.comments.CommentedMap()
                        )
                        twoVecCmd["Description"] = (
                            f"Two Vector Command {inds[0]}-{inds[1]}"
                        )
                        twoVecCmd["Index"] = inds
                        twoVecCmd["Primary Vector"] = pv = (
                            ruamel.yaml.comments.CommentedMap()
                        )
                        twoVecCmd["Secondary Vector"] = sv = (
                            ruamel.yaml.comments.CommentedMap()
                        )
                        for item in dsm["One Vector Configurations"]:
                            pvConf = item["One Vector"]
                            if pvConf["Index"] == inds[0]:
                                pv["Target"] = copy.deepcopy(
                                    pvConf["Primary Vector"]["Target"]
                                )
                                pv["Axis"] = copy.deepcopy(
                                    pvConf["Primary Vector"]["Axis"]
                                )
                                twoVecCmd["Controller"] = pvConf["Controller"]
                                twoVecCmd["Actuator"] = pvConf["Actuator"]
                                break
                        for item in dsm["Secondary Vector Configurations"]:
                            svConf = item["Secondary Vector"]
                            if svConf["Index"] == inds[1]:
                                sv["Target"] = copy.deepcopy(svConf["Target"])
                                sv["Axis"] = copy.deepcopy(svConf["Axis"])
                                break
                        dsm["Two Vector Configurations"].append(headDict)
                        cmd["Command Data"] = twoVecCmd

                elif "passive" not in subType.lower():
                    ind = cmd["Index"]
                    for seqItem in dsm[subType + " Configurations"]:
                        config = seqItem[subType]
                        if config["Index"] == ind:
                            cmd["Command Data"] = config
                            break

    del dsm["Secondary Vector Configurations"]

    for key in dsm.keys():
        if not key == "DSM Commands":
            for i, thing in enumerate(dsm[key]):
                thingName = list(thing.keys())[0]
                thing[thingName].yaml_set_anchor(thingName.replace(" ", "-") + f"_{i}")
                thing[thingName].anchor.always_dump = True

    for key in dsm.keys():
        if not key == "DSM Commands":
            for cmd in dsm[key]:
                thingName = list(cmd.keys())[0]
                cmdData = cmd[thingName]
                for cmdKey in cmdData.keys():
                    if cmdKey == "Controller" and key != "Controller Configurations":
                        test = cmdData[cmdKey]
                        for i, ctrl in enumerate(dsm["Controller Configurations"]):
                            if ctrl["Controller"]["Index"] == test:
                                cmdData[cmdKey] = ctrl["Controller"]
                    elif cmdKey == "Actuator":
                        test = cmdData[cmdKey]
                        for i, act in enumerate(dsm["Actuator Configurations"]):
                            if act["Actuator"]["Index"] == test:
                                cmdData[cmdKey] = act["Actuator"]
                    elif cmdKey == "Gains":
                        test = cmdData[cmdKey]
                        for i, gain in enumerate(dsm["Gains Configurations"]):
                            if gain["Gains"]["Index"] == test:
                                cmdData[cmdKey] = gain["Gains"]
                    elif cmdKey == "Limits":
                        test = cmdData[cmdKey]
                        for i, lim in enumerate(dsm["Limits Configurations"]):
                            if lim["Limits"]["Index"] == test:
                                cmdData[cmdKey] = lim["Limits"]
                    elif cmdKey == "Sensors":
                        for i, test in enumerate(cmdData[cmdKey]):
                            for sen in dsm["Sensor Configurations"]:
                                sen["Sensor"].yaml_set_anchor(
                                    "Sensor_%0d" % sen["Sensor"]["Index"]
                                )
                                sen["Sensor"].anchor.always_dump = True
                                if sen["Sensor"]["Index"] == test:
                                    cmdData[cmdKey][i] = sen["Sensor"]
                    elif cmdKey == "Sensor Set":
                        test = cmdData[cmdKey]
                        for senSet in dsm["Sensor Set Configurations"]:
                            senSet["Sensor Set"].yaml_set_anchor(
                                "Sensor_Set_%0d" % senSet["Sensor Set"]["Index"]
                            )
                            senSet["Sensor Set"].anchor.always_dump = True
                            if senSet["Sensor Set"]["Index"] == test:
                                cmdData[cmdKey] = senSet["Sensor Set"]
                    elif cmdKey == "Data":
                        for test in cmdData[cmdKey].keys():
                            for navData in dsm["Navigation Data Configurations"]:
                                navData["Navigation Data"].yaml_set_anchor(
                                    "Navigation_Data_%0d"
                                    % navData["Navigation Data"]["Index"]
                                )
                                navData["Navigation Data"].anchor.always_dump = True
                                if (
                                    navData["Navigation Data"]["Index"]
                                    == cmdData[cmdKey][test]
                                ):
                                    cmdData[cmdKey][test] = navData["Navigation Data"]

    nukeIndex(dsm)

    for lim in dsm["Sensor Configurations"]:
        del lim["Sensor"]["Index"]

    for lim in dsm["Sensor Set Configurations"]:
        del lim["Sensor Set"]["Index"]

    for lim in dsm["Navigation Data Configurations"]:
        del lim["Navigation Data"]["Index"]

    return dsm


# %%
def convertSim(missionDir, yaml, simFileName="Inp_Sim.txt", commentDict=None):
    sim = dict()
    with open(missionDir + "/InOut/" + simFileName, "r") as f:
        lines = f.readlines()

    confDict = sim["Simulation Control"] = dict()
    timeDict = sim["Time"] = dict()
    for lineNum, line in enumerate(lines[:8]):
        lineData = line.split("!")[0].strip()
        strData = lineData.split()
        match lineNum:
            case 2:
                confDict["Mode"] = lineData
            case 3:
                confDict["Duration"] = float(strData[0])
                confDict["Step Size"] = float(strData[1])
            case 4:
                confDict["File Interval"] = float(lineData)
            case 5:
                confDict["RNG Seed"] = int(lineData)
            case 6:
                confDict["Enable Graphics"] = lineData.lower() == "true"
            case 7:
                confDict["Command File"] = lineData

        offset = 9
        nOrbs = int(lines[offset].split("!")[0].strip())
        orbs = sim["Orbits"] = list()
        for i in range(nOrbs):
            newDict = dict()
            lineData = lines[offset + i + 1].split("!")[0].strip()
            strData = lineData.split()
            newDict["Name"] = ".".join(strData[1].split(".")[:-1])
            newDict["Enabled"] = strData[0].lower() == "true"
            orbs.append(newDict)

        offset += nOrbs + 2
        nSCs = int(lines[offset].split("!")[0].strip())
        scs = sim["SCs"] = list()
        for i in range(nSCs):
            newDict = dict()
            lineData = lines[offset + i + 1].split("!")[0].strip()
            strData = lineData.split()
            newDict["Name"] = ".".join(strData[2].split(".")[:-1])
            newDict["Orbit"] = orbs[int(strData[1])]["Name"]
            newDict["Enabled"] = strData[0].lower() == "true"
            scs.append(newDict)

        offset += nSCs + 1

        for lineNum, line in enumerate(lines[offset : offset + 22]):
            lineData = line.split("!")[0].strip()
            strData = lineData.split()
            match lineNum:
                case 1:
                    dateDict = timeDict["Date"] = dict()
                    dateDict["Month"] = int(strData[0])
                    dateDict["Day"] = int(strData[1])
                    dateDict["Year"] = int(strData[2])
                case 2:
                    dateDict = timeDict["Time"] = dict()
                    seconds = float(strData[2])
                    dateDict["Hour"] = int(strData[0])
                    dateDict["Minute"] = int(strData[1])
                    dateDict["Second"] = int(np.floor(seconds))
                    dateDict["Millisecond"] = int(
                        np.floor(seconds - dateDict["Second"]) * 1000
                    )
                case 3:
                    timeDict["Leap Seconds"] = float(lineData)
                case 4:
                    pertDict = sim["Perturbation Models"] = dict()
                    atmoDict = pertDict["Atmosphere"] = dict()
                    atmoDict["Enabled"] = "false"
                    atmoDict["Shadows"] = "false"
                    atmoList = atmoDict["Models"] = list()
                    tmpDict = dict()
                    atmoList.append(tmpDict)
                    tmpDict["World"] = "EARTH"
                    tmpDict["Method"] = lineData
                case 5:
                    tmpDict["F10.7"] = float(lineData)
                case 6:
                    tmpDict["Ap"] = float(lineData)
                case 7:
                    magDict = pertDict["Magnetic"] = dict()
                    magDict["Residual Mag Moment"] = "false"
                    magList = magDict["Models"] = list()
                    tmpDict = dict()
                    magList.append(tmpDict)
                    tmpDict["World"] = "EARTH"
                    tmpDict["Method"] = lineData
                case 8:
                    tmpDict["Degree"] = int(strData[0])
                    tmpDict["Order"] = int(strData[1])
                case 9:
                    gravDict = pertDict["Gravitation"] = dict()
                    gravDict["Enabled"] = "false"
                    gravDict["Gravity Gradient"] = "false"
                    gravPertList = gravDict["Models"] = list()
                    tmpDict = dict()
                    gravPertList.append(tmpDict)
                    tmpDict["World"] = "EARTH"
                    tmpDict["Degree"] = int(strData[0])
                    tmpDict["Order"] = int(strData[1])
                case 10:
                    tmpDict = dict()
                    gravPertList.append(tmpDict)
                    tmpDict["World"] = "MARS"
                    tmpDict["Degree"] = int(strData[0])
                    tmpDict["Order"] = int(strData[1])
                case 11:
                    tmpDict = dict()
                    gravPertList.append(tmpDict)
                    tmpDict["World"] = "LUNA"
                    tmpDict["Degree"] = int(strData[0])
                    tmpDict["Order"] = int(strData[1])
                case 12:
                    atmoDict["Enabled"] = strData[0].lower() == "true"
                    atmoDict["Shadows"] = strData[1].lower() == "true"
                case 13:
                    gravDict["Gravity Gradient"] = lineData.lower() == "true"
                case 14:
                    srpEnDict = pertDict["SRP"] = dict()
                    srpEnDict["Enabled"] = strData[0].lower() == "true"
                    srpEnDict["Shadows"] = strData[1].lower() == "true"
                case 15:
                    magDict["Residual Mag Moment"] = lineData.lower() == "true"
                case 16:
                    gravDict["Enabled"] = lineData.lower() == "true"
                case 17:
                    pertDict["Thruster Plume"] = lineData.lower() == "true"
                case 18:
                    pertDict["Contact"] = lineData.lower() == "true"
                case 19:
                    pertDict["CFD Slosh"] = lineData.lower() == "true"
                case 20:
                    pertDict["Albedo on CSS"] = lineData.lower() == "true"
                case 21:
                    pertDict["Output Env Torques to File"] = lineData.lower() == "true"

        offset += 22

        for lineNum, line in enumerate(lines[offset : offset + 12]):
            lineData = line.split("!")[0].strip()
            strData = lineData.split()
            match lineNum:
                case 1:
                    sim["Ephem Type"] = lineData
                case 2:
                    celDict = sim["Celestial Bodies"] = dict()
                    celDict["Mercury"] = lineData.lower() == "true"
                case 3:
                    celDict["Venus"] = lineData.lower() == "true"
                case 4:
                    celDict["Earth and Luna"] = lineData.lower() == "true"
                case 5:
                    celDict["Mars and its moons"] = lineData.lower() == "true"
                case 6:
                    celDict["Jupiter and its moons"] = lineData.lower() == "true"
                case 7:
                    celDict["Saturn and its moons"] = lineData.lower() == "true"
                case 8:
                    celDict["Uranus and its moons"] = lineData.lower() == "true"
                case 9:
                    celDict["Neptune and its moons"] = lineData.lower() == "true"
                case 10:
                    celDict["Pluto and its moons"] = lineData.lower() == "true"
                case 11:
                    celDict["Asteroids and Comets"] = lineData.lower() == "true"

        offset += 12

        for lineNum, line in enumerate(lines[offset : offset + 4]):
            lineData = line.split("!")[0].strip()
            strData = lineData.split()
            match lineNum:
                case 1:
                    lagDict = sim["Lagrange Systems"] = dict()
                    lagDict["Earth-Moon"] = lineData.lower() == "true"
                case 2:
                    lagDict["Sun-Earth"] = lineData.lower() == "true"
                case 3:
                    lagDict["Sun-Jupiter"] = lineData.lower() == "true"

        offset += 4
        nGS = int(lines[offset + 1].split("!")[0].strip())
        gss = sim["Ground Stations"] = list()
        if nGS > 0:
            for line in lines[offset + 2 : offset + 2 + nGS]:
                newDict = dict()
                newItem = {"Ground Station": newDict}
                lineData = line.split("!")[0].strip()
                strData = lineData.split()
                splitQuote = line.split('"')
                if len(splitQuote) == 2:
                    label = ""
                else:
                    label = splitQuote[1]
                newDict["Index"] = len(gss)
                newDict["Enabled"] = strData[0].lower() == "true"
                newDict["World"] = strData[1]
                newDict["Longitude"] = float(strData[2])
                newDict["Latitude"] = float(strData[3])
                newDict["Label"] = label
                gss.append(newItem)

    sim = convertToYamlAndComment(sim, commentDict)

    return sim


#  %%
def traverseAndSetNumericArray(data):
    if data:
        match data:
            case dict():
                for key in data.keys():
                    data[key] = traverseAndSetNumericArray(data[key])
            case list():
                if isinstance(data[0], (numbers.Number, str)):
                    data = ruamel.yaml.comments.CommentedSeq(data)
                    data.fa.set_flow_style()
                else:
                    for item in data:
                        item = traverseAndSetNumericArray(item)
            # case str():
            #     data = ruamel.yaml.scalarstring.DoubleQuotedScalarString(data)
    return data


def convertToYamlAndComment(data, commentDict):
    data = convertToYamlObjects(data)
    if commentDict is not None:
        for key in commentDict.keys():
            data.yaml_set_comment_before_after_key(key, after=commentDict[key])
        data.yaml_set_start_comment(commentDict["SOF"])
    return data


def convertToYamlObjects(data):
    if data:
        match data:
            case dict():
                for key in data.keys():
                    data[key] = convertToYamlObjects(data[key])
                data = ruamel.yaml.comments.CommentedMap(data)
            case list():
                if isinstance(data[0], (list, dict)):
                    for ind in range(len(data)):
                        data[ind] = convertToYamlObjects(data[ind])
                data = ruamel.yaml.comments.CommentedSeq(data)
    return data


comment_file_dict = {
    "SC_": "SC_comments.yaml",
    "Orb_": "Orb_comments.yaml",
    "Nodes_": "Nodes_comments.yaml",
    "Inp_DSM": "InpDSM_comments.yaml",
    "Inp_FOV": "InpFOV_comments.yaml",
    "Inp_Graphics": "InpGraphics_comments.yaml",
    "Inp_IPC": "InpIPC_comments.yaml",
    "Inp_NOS3": "InpNOS3_comments.yaml",
    "Inp_Region": "InpRegion_comments.yaml",
    "Inp_Sim": "InpSim_comments.yaml",
    "Inp_TDRS": "InpTDRS_comments.yaml",
}
convert_func_dict = {
    # "SC_": convertSC,
    "Orb_": convertOrb,
    "Nodes_": convertNodes,
    "Inp_DSM": convertDSM,
    "Inp_FOV": convertFOV,
    "Inp_Graphics": convertGraphics,
    "Inp_IPC": convertIPC,
    "Inp_NOS3": convertNOS3,
    "Inp_Region": convertRegion,
    # "Inp_Sim": convertSim,
    "Inp_TDRS": convertTDRS,
}
f_type_list = [
    # "SC_",
    "Orb_",
    "Nodes_",
    "Inp_DSM",
    "Inp_FOV",
    "Inp_Graphics",
    "Inp_IPC",
    "Inp_NOS3",
    "Inp_Region",
    # "Inp_Sim",
    "Inp_TDRS",
]
startswith_type = {
    "SC_",
    "Orb_",
    "Nodes_",
}

# %%
if __name__ == "__main__":
    yaml_file = sys.argv[1]
    missionDir = sys.argv[2]
    doComment = sys.argv[3] == "True"

    comment_dir = "yaml/yamlComments/"
    yaml = YAML()
    yaml.version = (1, 2)
    yaml.indent(
        mapping=2,
        sequence=4,
        offset=2,
    )
    yaml.preserve_quotes = True
    with open(yaml_file) as file:
        yaml_data = yaml.load(file)

    files = os.listdir(missionDir + "/InOut/")
    for file in files:
        splitFile = file.split(".")
        if splitFile[-1] == "txt":
            data = []
            name = splitFile[0]
            for f_type in f_type_list:
                if f_type in startswith_type and name.startswith(f_type):
                    comment_file_name = comment_file_dict[f_type]
                    if doComment and comment_file_name is not None:
                        with open(comment_dir + comment_file_name) as comm_file:
                            yaml_comments = yaml.load(comm_file)
                    else:
                        yaml_comments = None
                    data = convert_func_dict[f_type](
                        missionDir, yaml, file, commentDict=yaml_comments
                    )
                elif f_type == name:
                    comment_file_name = comment_file_dict[f_type]
                    if doComment and comment_file_name is not None:
                        with open(comment_dir + comment_file_name) as comm_file:
                            yaml_comments = yaml.load(comm_file)
                    else:
                        yaml_comments = None
                    data = convert_func_dict[f_type](
                        missionDir, yaml, commentDict=yaml_comments
                    )

            if name != "Inp_Cmd" and not len(data) == 0:
                yamlName = name + ".yaml"
                data = traverseAndSetNumericArray(data)
                with open(missionDir + "/InOut/" + yamlName, "w") as f:
                    yaml.dump(data, f)

# %%
