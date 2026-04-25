/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __42DEFINES_H__
#define __42DEFINES_H__

#define TRUE  1
#define FALSE 0

#define ON  1
#define OFF 0

#define AU2m 149597870000.0

#define ABS(X) ((X) > 0 ? (X) : (-X))

#define IDX2(x, y, Ny)        ((x) * (Ny) + (y))
#define IDX3(x, y, z, Ny, Nz) (((x) * (Ny) + (y)) * (Nz) + (z))
#define IDX4(x, y, z, w, Ny, Nz, Nw)                                           \
   ((((x) * (Ny) + (y)) * (Nz) + (z)) * (Nw) + (w))

#define POSITIVE 1
#define NEGATIVE 0

#define DIR_CW  0
#define DIR_CCW 1

#define EARTHMOON  0
#define SUNEARTH   1
#define SUNJUPITER 2

#define LAGPT_L1 0
#define LAGPT_L2 1
#define LAGPT_L3 2
#define LAGPT_L4 3
#define LAGPT_L5 4

#define NONE   0
#define DIPOLE 1
#define IGRF   2

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define POS_X 0
#define POS_Y 1
#define POS_Z 2
#define NEG_X 3
#define NEG_Y 4
#define NEG_Z 5

/* POV Frames */
#define FRAME_N 0
#define FRAME_L 1
#define FRAME_F 2
#define FRAME_S 3
#define FRAME_B 4

/* POV (Host and) Target Types */
#define TARGET_WORLD    0
#define TARGET_REFORB   1
#define TARGET_FRM      2
#define TARGET_SC       3
#define TARGET_BODY     4
#define TARGET_TDRS     5
#define TARGET_VELOCITY 7
#define TARGET_LAGPT    8
#define TARGET_MAGFIELD 9
#define TARGET_VEC      10

/* POV Modes */
#define TRACK_HOST    0
#define TRACK_TARGET  1
#define FIXED_IN_HOST 2

/* POV Views */
#define VIEW_FRONT       8
#define VIEW_FRONT_RIGHT 9
#define VIEW_RIGHT       6
#define VIEW_REAR_RIGHT  3
#define VIEW_REAR        2
#define VIEW_REAR_LEFT   1
#define VIEW_LEFT        4
#define VIEW_FRONT_LEFT  7
#define VIEW_UP          5
#define VIEW_DOWN        0

#define FOV_WIREFRAME 0
#define FOV_SOLID     1
#define FOV_VECTOR    2
#define FOV_PLANE     3

#define LAGDOF_MODES  0
#define LAGDOF_COWELL 1
#define LAGDOF_SPLINE 2

#define DYN_GAUSS_ELIM 0
#define DYN_ORDER_N    1

#define ORBDOF_FIXED      0
#define ORBDOF_EULER_HILL 1
#define ORBDOF_ENCKE      2
#define ORBDOF_COWELL     3

#define REFPT_CM    0
#define REFPT_JOINT 1

/* World Visibility */
#define WORLD_IS_TOO_SMALL_TO_SEE 0
#define WORLD_IS_POINT_SIZED      1
#define WORLD_SHOWS_DISK          2
#define WORLD_IS_SUN              3
#define WORLD_IS_OUT_OF_POV       4
#define WORLD_IS_NEAR             5

/* Time Modes */
#define FAST_TIME     0
#define REAL_TIME     1
#define EXTERNAL_TIME 2
#define NOS3_TIME     3

/* World Types */
#define SUN      0
#define PLANET   1
#define MOON     2
#define ASTEROID 3
#define COMET    4

/* Command Types */
#define CMD_DIRECTION 0
#define CMD_TARGET    1

/* Command Parameter Types */
#define PARM_EULER_ANGLES 0
#define PARM_QUATERNION   1
#define PARM_VECTORS      2
#define PARM_AXIS_SPIN    3
#define PARM_MIRROR       4
#define PARM_DETUMBLE     5
#define PARM_UNITVECTOR   6

/* ProxOps View */
#define VIEW_SIDE 6
#define VIEW_TOP  7

/* Solar Activity Modes */
#define TWOSIGMA_ATMO 0
#define NOMINAL_ATMO  1
#define USER_ATMO     2

/* Modes for InterProcess Comm */
#define IPC_OFF       0
#define IPC_TX        1
#define IPC_RX        2
#define IPC_TXRX      3
#define IPC_ACS       4
#define IPC_WRITEFILE 5
#define IPC_READFILE  6
#define IPC_SPIRENT   7
#define IPC_FFTB      8

/* Socket Roles for InterProcess Comm */
#define IPC_SERVER       0
#define IPC_CLIENT       1
#define IPC_GMSEC_CLIENT 2

/* Secs from J2000 to the Unix epoch of 1 Jan 1970 */
#define UNIX_EPOCH (-946728000.0)
/* Secs from J2000 to the GPS epoch of 6 Jan 1980 */
#define GPS_EPOCH (-630763200.0)

/* Constellation Classes */
#define NUM_CONSTELL    88
#define MAJOR_CONSTELL  0
#define ZODIAC_CONSTELL 1
#define MINOR_CONSTELL  2

/* Joint Types */
#define PASSIVE_JOINT            0
#define ACTUATED_JOINT           1
#define STEPPER_MOTOR_JOINT      2
#define VIBRATION_ISOLATOR_JOINT 3
#define SLOSH_JOINT              4
#define STEERING_MIRROR_JOINT    5
#define TVC_JOINT                6
#define AD_HOC_JOINT             7

/* For Shakers */
#define FORCE  0
#define TORQUE 1

/* Thruster Command Modes */
#define THR_PULSED       0
#define THR_PROPORTIONAL 1

/* Optics */
#define OPT_APERTURE 0
#define OPT_PLANAR   1
#define OPT_CONIC    2
#define OPT_THINLENS 3
#define OPT_DETECTOR 4
#define OPT_CONCAVE  (1.0)
#define OPT_CONVEX   (-1.0)

#endif /* __42DEFINES_H__ */
