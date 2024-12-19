##########################  Macro Definitions  ############################

# Let's try to auto-detect what platform we're on.
# If this fails, set 42PLATFORM manually in the else block.
AUTOPLATFORM = Failed
ifeq ($(MSYSTEM),MINGW32)
   AUTOPLATFORM = Succeeded
   42PLATFORM = __MSYS__
endif
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Linux)
   AUTOPLATFORM = Succeeded
   42PLATFORM = __linux__
endif
ifeq ($(UNAME_S),Darwin)
   AUTOPLATFORM = Succeeded
   42PLATFORM = __APPLE__
endif
ifeq ($(AUTOPLATFORM),Failed)
   # Autodetect failed.  Set platform manually.
   #42PLATFORM = __APPLE__
   #42PLATFORM = __linux__
   42PLATFORM = __MSYS__
endif


GUIFLAG = -D _ENABLE_GUI_
#GUIFLAG =

SHADERFLAG = -D _USE_SHADERS_
#SHADERFLAG =

CFDFLAG =
#CFDFLAG = -D _ENABLE_CFD_SLOSH_

FFTBFLAG =
#FFTBFLAG = -D _ENABLE_FFTB_CODE_

GSFCFLAG =
#GSFCFLAG = -D _USE_GSFC_WATERMARK_

STANDALONEFLAG =
#STANDALONEFLAG = -D _AC_STANDALONE_

GMSECFLAG =
#GMSECFLAG = -D _ENABLE_GMSEC_

RBTFLAG =
#RBTFLAG = -D _ENABLE_RBT_

DEBUGFLAG =
# DEBUGFLAG = -D _DEBUG_GRAV_ -D _DEBUG_MAG_

SPICEFLAG = -D _ENABLE_SPICE_
# SPICEFLAG =

ifeq ($(strip $(GMSECFLAG)),)
   GMSECDIR =
   GMSECINC =
   GMSECBIN =
   GMSECLIB =
else
   GMSECDIR = ~/GMSEC/
   GMSECINC = -I $(GMSECDIR)include/
   GMSECBIN = -L $(GMSECDIR)bin/
   GMSECLIB = -lGMSECAPI
endif

# Basic directories
mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
current_dir := $(patsubst %/,%,$(dir $(mkfile_path)))
HOMEDIR = ./
PROJDIR = ./
KITDIR = $(PROJDIR)Kit/
OBJ = $(PROJDIR)Object/
INC = $(PROJDIR)Include/
TESTS = $(PROJDIR)Tests/
DATAFILTER = $(PROJDIR)data_filter/
SRC = $(PROJDIR)Source/
KITINC = $(KITDIR)Include/
KITSRC = $(KITDIR)Source/
INOUT = $(PROJDIR)InOut/
GSFCSRC = $(PROJDIR)/GSFC/Source/
IPCSRC = $(SRC)IPC/

# Use conda if we got it
ifeq (,$(shell which conda))
   LDFLAGS =
else
   CONDA_DIR=$(shell echo $(CONDA_PREFIX))
   CONDA_LIB_DIR = $(CONDA_DIR)/lib
   LDFLAGS="-Wl,-rpath,$(CONDA_LIB_DIR)"
endif
# CSPICE library
CSPICEDIR = $(current_dir)/cspice/
CSPICEINC = $(CSPICEDIR)include/
CSPICESRC = $(CSPICEDIR)src/
CSPICELIB = $(CSPICEDIR)lib/

ifeq ($(strip $(SPICEFLAG)),)
   SPICEFLAGS =
   SPICELIBFLAGS =
else
   SPICEFLAGS =  -I $(CSPICEINC)
   SPICELIBFLAGS = -L $(CSPICELIB)
endif

ifeq ($(42PLATFORM),__APPLE__)
   # Mac Macros
   CINC = -I /usr/include -I /usr/local/include
   EXTERNDIR =
   # ARCHFLAG = -arch i386
   # ARCHFLAG = -arch x86_64
   ARCHFLAG = -arch arm64
   # For graphics interface, choose GLUT or GLFW GUI libraries
   # GLUT is well known, but GLFW is better for newer Mac's hires displays
   # OSX fixed their hires GLUT issue.  Keep GLFW around just in case.
   #GLUT_OR_GLFW = _USE_GLFW_
   GLUT_OR_GLFW = _USE_GLUT_

   LFLAGS =
   ifneq ($(strip $(GUIFLAG)),)
      GLINC = -I /System/Library/Frameworks/OpenGL.framework/Headers/ -I /System/Library/Frameworks/GLUT.framework/Headers/
      ifeq ($(strip $(GLUT_OR_GLFW)),_USE_GLUT_)
         LIBS = -framework System -framework Carbon -framework OpenGL -framework GLUT
         GUIOBJ = $(OBJ)42gl.o $(OBJ)42glut.o $(OBJ)glkit.o $(OBJ)42gpgpu.o
         GUI_LIB = -D _USE_GLUT_
      else
         LIBS = -lglfw -framework System -framework Carbon -framework OpenGL -framework GLUT
         GUIOBJ = $(OBJ)42gl.o $(OBJ)42glfw.o $(OBJ)glkit.o $(OBJ)42gpgpu.o
         GUI_LIB = -D _USE_GLFW_
      endif
   else
      GLINC =
      LIBS =
      GUIOBJ =
   endif

   ifneq ($(strip $(SPICEFLAG)),)
      # g++ requires the lib prefix, so make a symbolic link to do this
      $(shell ln -s $(CSPICELIB)cspice.a $(CSPICELIB)libcspice.a)
      SPICELIBFLAGS += -lcspice
   endif

   XWARN =
   EXENAME = deepthought
   CC = gcc
endif

ifeq ($(42PLATFORM),__linux__)
   # Linux Macros
   CINC =
   EXTERNDIR =
   ARCHFLAG =
   # For graphics interface, choose GLUT or GLFW GUI libraries
   # GLUT is well known, but GLFW is better for newer Mac's hires displays
   #GLUT_OR_GLFW = _USE_GLFW_
   GLUT_OR_GLFW = _USE_GLUT_

   ifneq ($(strip $(GUIFLAG)),)
      ifeq ($(strip $(GLUT_OR_GLFW)),_USE_GLUT_)
         GUIOBJ = $(OBJ)42gl.o $(OBJ)42glut.o $(OBJ)glkit.o $(OBJ)42gpgpu.o
         LIBS = -lglut -lGLU -lGL -ldl -lm -lpthread
         GLINC = -I /usr/include/GL/
         LFLAGS = -L $(KITDIR)/GL/lib/
         GUI_LIB = -D _USE_GLUT_
      else
         GUIOBJ = $(OBJ)42gl.o $(OBJ)42glfw.o $(OBJ)glkit.o $(OBJ)42gpgpu.o
         LIBS = -lglfw -lglut -lGLU -lGL -ldl -lm -lpthread
         GLINC = -I /usr/include/GL/ -I /usr/include/GLFW
         GUI_LIB = -D _USE_GLFW_
      endif
   else
      GUIOBJ =
      GLINC =
      LIBS = -ldl -lm -lpthread
      LFLAGS =
   endif

   ifneq ($(strip $(SPICEFLAG)),)
      SPICELIBFLAGS += -l:cspice.a
   endif

   XWARN = -Wno-unused-variable -Wno-unused-but-set-variable -Wno-stringop-overread
   EXENAME = deepthought
   CC = gcc
endif

ifeq ($(42PLATFORM),__MSYS__)
   CINC =
   EXTERNDIR = /c/42ExternalSupport/
   # For graphics interface, choose GLUT or GLFW GUI libraries
   # GLUT is well known, but GLFW is better for newer Mac's hires displays
   #GLUT_OR_GLFW = _USE_GLFW_
   GLUT_OR_GLFW = _USE_GLUT_

   ifneq ($(strip $(GUIFLAG)),)
      # TODO: Option to use GLFW instead of GLUT?
      GLEW = $(EXTERNDIR)GLEW/
      GLUT = $(EXTERNDIR)freeglut/
      LIBS =  -lopengl32 -lglu32 -lfreeglut -lws2_32 -lglew32
      LFLAGS = -L $(GLUT)lib/ -L $(GLEW)lib/
      GUIOBJ = $(OBJ)42gl.o $(OBJ)42glut.o $(OBJ)glkit.o $(OBJ)42gpgpu.o
      GLINC = -I $(GLEW)include/GL/ -I $(GLUT)include/GL/
      ARCHFLAG = -D GLUT_NO_LIB_PRAGMA -D GLUT_NO_WARNING_DISABLE -D GLUT_DISABLE_ATEXIT_HACK
   else
      GUIOBJ =
      GLINC =
      LIBS =  -lws2_32
      LFLAGS =
      ARCHFLAG =
   endif
   XWARN =
   EXENAME = deepthought.exe
   CC = gcc
endif

# If not using GUI, don't compile GUI-related files
ifeq ($(strip $(GUIFLAG)),)
   GUIOBJ =
endif

# If not in FFTB, don't compile FFTB-related files
ifneq ($(strip $(FFTBFLAG)),)
   FFTBOBJ = $(OBJ)42fftb.o
else
   FFTBOBJ =
endif

ifneq ($(strip $(CFDFLAG)),)
   SLOSHOBJ = $(OBJ)42CfdSlosh.o
else
   SLOSHOBJ =
endif

# If not _AC_STANDALONE_, link AcApp.c in with the rest of 42
ifneq ($(strip $(STANDALONEFLAG)),)
   ACOBJ =
else
   ACOBJ = $(OBJ)AcApp.o
endif

ifneq ($(strip $(RBTFLAG)),)
   RBTDIR = $(PROJDIR)../../GSFC/RBT/
   RBTSRC = $(RBTDIR)Source/
   RBTOBJ = $(OBJ)RbtFsw.o
else
   RBTDIR =
   RBTSRC =
   RBTOBJ =
endif


ifneq ($(strip $(GMSECFLAG)),)
   GMSECOBJ = $(OBJ)gmseckit.o
   ACIPCOBJ = $(OBJ)AppReadFromFile.o $(OBJ)AppWriteToGmsec.o $(OBJ)AppReadFromGmsec.o \
      $(OBJ)AppWriteToSocket.o $(OBJ)AppReadFromSocket.o $(OBJ)AppWriteToFile.o
   SIMIPCOBJ = $(OBJ)SimWriteToFile.o $(OBJ)SimWriteToGmsec.o $(OBJ)SimWriteToSocket.o \
      $(OBJ)SimReadFromFile.o $(OBJ)SimReadFromGmsec.o $(OBJ)SimReadFromSocket.o
else
   GMSECOBJ =
   ACIPCOBJ = $(OBJ)AppReadFromFile.o \
      $(OBJ)AppWriteToSocket.o $(OBJ)AppReadFromSocket.o $(OBJ)AppWriteToFile.o
   SIMIPCOBJ = $(OBJ)SimWriteToFile.o $(OBJ)SimWriteToSocket.o \
      $(OBJ)SimReadFromFile.o $(OBJ)SimReadFromSocket.o
endif

42OBJ = $(OBJ)42main.o $(OBJ)42exec.o $(OBJ)42actuators.o $(OBJ)42cmd.o \
$(OBJ)42dynamics.o $(OBJ)42environs.o $(OBJ)42ephem.o $(OBJ)42fsw.o \
$(OBJ)42init.o $(OBJ)42ipc.o $(OBJ)42jitter.o $(OBJ)42joints.o \
$(OBJ)42optics.o $(OBJ)42perturb.o $(OBJ)42report.o $(OBJ)42sensors.o \
$(OBJ)42nos3.o $(OBJ)42dsm.o

KITOBJ = $(OBJ)dcmkit.o $(OBJ)envkit.o $(OBJ)fswkit.o $(OBJ)geomkit.o \
$(OBJ)iokit.o $(OBJ)mathkit.o $(OBJ)nrlmsise00kit.o \
$(OBJ)orbkit.o $(OBJ)radbeltkit.o $(OBJ)sigkit.o $(OBJ)sphkit.o $(OBJ)timekit.o \
$(OBJ)docoptkit.o $(OBJ)dsmkit.o $(OBJ)navkit.o

LIBKITOBJ = $(OBJ)dcmkit.o $(OBJ)envkit.o $(OBJ)fswkit.o $(OBJ)geomkit.o \
$(OBJ)iokit.o $(OBJ)mathkit.o $(OBJ)orbkit.o $(OBJ)sigkit.o $(OBJ)sphkit.o $(OBJ)timekit.o

ACKITOBJ = $(OBJ)dcmkit.o $(OBJ)mathkit.o $(OBJ)fswkit.o $(OBJ)iokit.o $(OBJ)timekit.o

ACIPCOBJ = $(OBJ)AppReadFromFile.o \
$(OBJ)AppWriteToSocket.o $(OBJ)AppReadFromSocket.o $(OBJ)AppWriteToFile.o

TESTOBJ = $(OBJ)tests.o $(OBJ)mathkit_tests.o $(OBJ)navkit_tests.o\
$(OBJ)test_lib.o $(OBJ)42exec.o $(OBJ)42actuators.o $(OBJ)42cmd.o \
$(OBJ)42dynamics.o $(OBJ)42environs.o $(OBJ)42ephem.o $(OBJ)42fsw.o \
$(OBJ)42init.o $(OBJ)42ipc.o $(OBJ)42jitter.o $(OBJ)42joints.o \
$(OBJ)42perturb.o $(OBJ)42report.o $(OBJ)42sensors.o \
$(OBJ)42nos3.o $(OBJ)42dsm.o

#ANSIFLAGS = -Wstrict-prototypes -pedantic -ansi -Werror
ANSIFLAGS =

CFLAGS = -fpic -Wall -Wshadow -Wno-deprecated $(XWARN) -g  $(ANSIFLAGS) $(GLINC) $(CINC) -I $(INC) -I $(KITINC) -I $(KITSRC) -I $(RBTSRC) $(GMSECINC) -O0 $(ARCHFLAG) $(GUIFLAG) $(GUI_LIB) $(SHADERFLAG) $(CFDFLAG) $(FFTBFLAG) $(GSFCFLAG) $(GMSECFLAG) $(STANDALONEFLAG) $(RBTFLAG) $(SPICEFLAG) $(DEBUGFLAG)

CFLAGS+= `pkg-config --cflags libfyaml`
LFLAGS+= `pkg-config --libs libfyaml`

##########################  Rules to link 42  #############################

deepthought : $(42OBJ) $(GUIOBJ) $(SIMIPCOBJ) $(FFTBOBJ) $(SLOSHOBJ) $(KITOBJ) $(ACOBJ) $(GMSECOBJ) $(RBTOBJ)
	$(CC) $(LFLAGS) $(SPICEFLAGS) $(LDFLAGS) $(GMSECBIN) -o $(EXENAME) $(42OBJ) $(GUIOBJ) $(FFTBOBJ) $(SLOSHOBJ) $(KITOBJ) $(ACOBJ) $(GMSECOBJ) $(SIMIPCOBJ) $(RBTOBJ) $(LIBS) $(GMSECLIB) $(SPICELIBFLAGS)

Test : $(TESTOBJ) $(GUIOBJ) $(SIMIPCOBJ) $(FFTBOBJ) $(SLOSHOBJ) $(KITOBJ) $(ACOBJ) $(GMSECOBJ) $(RBTOBJ)
	$(CC) $(LFLAGS) $(SPICEFLAGS) $(LDFLAGS) -o Test $(TESTOBJ) $(GUIOBJ) $(FFTBOBJ) $(SLOSHOBJ) $(KITOBJ) $(ACOBJ) $(GMSECOBJ) $(SIMIPCOBJ) $(RBTOBJ) $(LIBS) $(GMSECLIB) $(SPICELIBFLAGS)

AcApp : $(OBJ)AcApp.o $(ACKITOBJ) $(ACIPCOBJ) $(GMSECOBJ)
	$(CC) $(LFLAGS) $(LDFLAGS) -o AcApp $(OBJ)AcApp.o $(ACKITOBJ) $(ACIPCOBJ) $(GMSECOBJ) $(LIBS)

42kit : $(LIBKITOBJ)
	$(CC) $(LFLAGS) $(LDFLAGS) -shared -o $(KITDIR)42kit.so $(LIBKITOBJ)


####################  Rules to compile objects  ###########################

$(OBJ)tests.o       : $(TESTS)tests.c $(TESTS)mathkit_tests.h
	$(CC) $(CFLAGS) -c $(TESTS)tests.c -o $(OBJ)tests.o

$(OBJ)mathkit_tests.o: $(TESTS)mathkit_tests.c $(KITINC)mathkit.h
	$(CC) $(CFLAGS) -c $(TESTS)mathkit_tests.c -o $(OBJ)mathkit_tests.o

$(OBJ)navkit_tests.o: $(TESTS)navkit_tests.c $(INC)DSMTypes.h $(KITINC)navkit.h
	$(CC) $(CFLAGS) -c $(TESTS)navkit_tests.c -o $(OBJ)navkit_tests.o

$(OBJ)test_lib.o: $(TESTS)test_lib.c
	$(CC) $(CFLAGS) -c $(TESTS)test_lib.c -o $(OBJ)test_lib.o

$(OBJ)42main.o      : $(SRC)42main.c
	$(CC) $(CFLAGS) -c $(SRC)42main.c -o $(OBJ)42main.o

$(OBJ)42exec.o      : $(SRC)42exec.c $(INC)42.h
	$(CC) $(CFLAGS) -c $(SRC)42exec.c -o $(OBJ)42exec.o

$(OBJ)42actuators.o : $(SRC)42actuators.c $(INC)42.h $(INC)Ac.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(SRC)42actuators.c -o $(OBJ)42actuators.o

$(OBJ)42cmd.o : $(SRC)42cmd.c $(INC)42.h $(INC)Ac.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(SRC)42cmd.c -o $(OBJ)42cmd.o

$(OBJ)42dynamics.o  : $(SRC)42dynamics.c $(INC)42.h
	$(CC) $(CFLAGS) -c $(SRC)42dynamics.c -o $(OBJ)42dynamics.o

$(OBJ)42environs.o  : $(SRC)42environs.c $(INC)42.h
	$(CC) $(CFLAGS) -c $(SRC)42environs.c -o $(OBJ)42environs.o

$(OBJ)42ephem.o     : $(SRC)42ephem.c $(INC)42.h
	$(CC) $(CFLAGS) $(SPICEFLAGS) -c $(SRC)42ephem.c -o $(OBJ)42ephem.o

$(OBJ)42fsw.o       : $(SRC)42fsw.c $(INC)Ac.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(SRC)42fsw.c -o $(OBJ)42fsw.o

$(OBJ)42gl.o        : $(SRC)42gl.c $(INC)42.h $(INC)42gl.h
	$(CC) $(CFLAGS) -c $(SRC)42gl.c -o $(OBJ)42gl.o

$(OBJ)42glfw.o	: $(SRC)42glfw.c $(INC)42.h $(INC)42gl.h $(INC)42glfw.h
	$(CC) $(CFLAGS) -c $(SRC)42glfw.c -o $(OBJ)42glfw.o

$(OBJ)42glut.o      : $(SRC)42glut.c $(INC)42.h $(INC)42gl.h $(INC)42glut.h
	$(CC) $(CFLAGS) -c $(SRC)42glut.c -o $(OBJ)42glut.o

$(OBJ)42gpgpu.o      : $(SRC)42gpgpu.c $(INC)42.h $(INC)42gl.h $(INC)42glut.h
	$(CC) $(CFLAGS) -c $(SRC)42gpgpu.c -o $(OBJ)42gpgpu.o

$(OBJ)42init.o      : $(SRC)42init.c $(INC)42.h
	$(CC) $(CFLAGS) $(SPICEFLAGS) -c $(SRC)42init.c -o $(OBJ)42init.o

$(OBJ)42ipc.o       : $(SRC)42ipc.c $(INC)42.h
	$(CC) $(CFLAGS) -c $(SRC)42ipc.c -o $(OBJ)42ipc.o

$(OBJ)42jitter.o    : $(SRC)42jitter.c $(INC)42.h
	$(CC) $(CFLAGS) -c $(SRC)42jitter.c -o $(OBJ)42jitter.o

$(OBJ)42joints.o    : $(SRC)42joints.c $(INC)42.h
	$(CC) $(CFLAGS) -c $(SRC)42joints.c -o $(OBJ)42joints.o

$(OBJ)42optics.o   : $(SRC)42optics.c $(INC)42.h
	$(CC) $(CFLAGS) -c $(SRC)42optics.c -o $(OBJ)42optics.o

$(OBJ)42perturb.o   : $(SRC)42perturb.c $(INC)42.h
	$(CC) $(CFLAGS) -c $(SRC)42perturb.c -o $(OBJ)42perturb.o

$(OBJ)42report.o    : $(SRC)42report.c $(INC)42.h
	$(CC) $(CFLAGS) $(SPICEFLAGS) -c $(SRC)42report.c -o $(OBJ)42report.o

$(OBJ)42sensors.o   : $(SRC)42sensors.c $(INC)42.h $(INC)Ac.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(SRC)42sensors.c -o $(OBJ)42sensors.o

$(OBJ)dcmkit.o      : $(KITSRC)dcmkit.c
	$(CC) $(CFLAGS) -c $(KITSRC)dcmkit.c -o $(OBJ)dcmkit.o

$(OBJ)envkit.o      : $(KITSRC)envkit.c
	$(CC) $(CFLAGS) -c $(KITSRC)envkit.c -o $(OBJ)envkit.o

$(OBJ)fswkit.o      : $(KITSRC)fswkit.c
	$(CC) $(CFLAGS) -c $(KITSRC)fswkit.c -o $(OBJ)fswkit.o

$(OBJ)dsmkit.o      : $(KITSRC)dsmkit.c
	$(CC) $(CFLAGS) -c $(KITSRC)dsmkit.c -o $(OBJ)dsmkit.o

$(OBJ)navkit.o      : $(KITSRC)navkit.c
	$(CC) $(CFLAGS) -c $(KITSRC)navkit.c -o $(OBJ)navkit.o

$(OBJ)glkit.o      : $(KITSRC)glkit.c $(KITINC)glkit.h
	$(CC) $(CFLAGS) -c $(KITSRC)glkit.c -o $(OBJ)glkit.o

$(OBJ)geomkit.o      : $(KITSRC)geomkit.c $(KITINC)geomkit.h
	$(CC) $(CFLAGS) -c $(KITSRC)geomkit.c -o $(OBJ)geomkit.o

$(OBJ)gmseckit.o      : $(KITSRC)gmseckit.c $(KITINC)gmseckit.h
	$(CC) $(CFLAGS) -c $(KITSRC)gmseckit.c -o $(OBJ)gmseckit.o

$(OBJ)iokit.o      : $(KITSRC)iokit.c
	$(CC) $(CFLAGS) -c $(KITSRC)iokit.c -o $(OBJ)iokit.o

$(OBJ)mathkit.o     : $(KITSRC)mathkit.c
	$(CC) $(CFLAGS) -c $(KITSRC)mathkit.c -o $(OBJ)mathkit.o

$(OBJ)nrlmsise00kit.o   : $(KITSRC)nrlmsise00kit.c
	$(CC) $(CFLAGS) -c $(KITSRC)nrlmsise00kit.c -o $(OBJ)nrlmsise00kit.o

$(OBJ)orbkit.o      : $(KITSRC)orbkit.c
	$(CC) $(CFLAGS) -c $(KITSRC)orbkit.c -o $(OBJ)orbkit.o

$(OBJ)radbeltkit.o      : $(KITSRC)radbeltkit.c
	$(CC) $(CFLAGS) -c $(KITSRC)radbeltkit.c -o $(OBJ)radbeltkit.o

$(OBJ)sigkit.o      : $(KITSRC)sigkit.c
	$(CC) $(CFLAGS) -c $(KITSRC)sigkit.c -o $(OBJ)sigkit.o

$(OBJ)sphkit.o      : $(KITSRC)sphkit.c
	$(CC) $(CFLAGS) -c $(KITSRC)sphkit.c -o $(OBJ)sphkit.o

$(OBJ)timekit.o     : $(KITSRC)timekit.c
	$(CC) $(CFLAGS) -c $(KITSRC)timekit.c -o $(OBJ)timekit.o

$(OBJ)42CfdSlosh.o      : $(GSFCSRC)42CfdSlosh.c $(INC)42.h
	$(CC) $(CFLAGS) -c $(GSFCSRC)42CfdSlosh.c -o $(OBJ)42CfdSlosh.o

$(OBJ)42fftb.o         : $(GSFCSRC)42fftb.c $(INC)42.h
	$(CC) $(CFLAGS) -c $(GSFCSRC)42fftb.c -o $(OBJ)42fftb.o

$(OBJ)AcApp.o          : $(SRC)AcApp.c $(INC)Ac.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(SRC)AcApp.c -o $(OBJ)AcApp.o

$(OBJ)SimWriteToFile.o  : $(IPCSRC)SimWriteToFile.c $(INC)42.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(IPCSRC)SimWriteToFile.c -o $(OBJ)SimWriteToFile.o

$(OBJ)SimWriteToGmsec.o  : $(IPCSRC)SimWriteToGmsec.c $(INC)42.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(IPCSRC)SimWriteToGmsec.c -o $(OBJ)SimWriteToGmsec.o

$(OBJ)SimWriteToSocket.o  : $(IPCSRC)SimWriteToSocket.c $(INC)42.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(IPCSRC)SimWriteToSocket.c -o $(OBJ)SimWriteToSocket.o

$(OBJ)SimReadFromFile.o  : $(IPCSRC)SimReadFromFile.c $(INC)42.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(IPCSRC)SimReadFromFile.c -o $(OBJ)SimReadFromFile.o

$(OBJ)SimReadFromGmsec.o  : $(IPCSRC)SimReadFromGmsec.c $(INC)42.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(IPCSRC)SimReadFromGmsec.c -o $(OBJ)SimReadFromGmsec.o

$(OBJ)SimReadFromSocket.o  : $(IPCSRC)SimReadFromSocket.c $(INC)42.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(IPCSRC)SimReadFromSocket.c -o $(OBJ)SimReadFromSocket.o

#$(OBJ)SimReadFromCmd.o  : $(IPCSRC)SimReadFromCmd.c $(INC)42.h $(INC)AcTypes.h
#	$(CC) $(CFLAGS) -c $(IPCSRC)SimReadFromCmd.c -o $(OBJ)SimReadFromCmd.o

$(OBJ)AppWriteToFile.o  : $(IPCSRC)AppWriteToFile.c $(INC)42.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(IPCSRC)AppWriteToFile.c -o $(OBJ)AppWriteToFile.o

$(OBJ)AppWriteToGmsec.o  : $(IPCSRC)AppWriteToGmsec.c $(INC)42.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(IPCSRC)AppWriteToGmsec.c -o $(OBJ)AppWriteToGmsec.o

$(OBJ)AppWriteToSocket.o  : $(IPCSRC)AppWriteToSocket.c $(INC)42.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(IPCSRC)AppWriteToSocket.c -o $(OBJ)AppWriteToSocket.o

$(OBJ)AppReadFromFile.o  : $(IPCSRC)AppReadFromFile.c $(INC)42.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(IPCSRC)AppReadFromFile.c -o $(OBJ)AppReadFromFile.o

$(OBJ)AppReadFromGmsec.o  : $(IPCSRC)AppReadFromGmsec.c $(INC)42.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(IPCSRC)AppReadFromGmsec.c -o $(OBJ)AppReadFromGmsec.o

$(OBJ)AppReadFromSocket.o  : $(IPCSRC)AppReadFromSocket.c $(INC)42.h $(INC)AcTypes.h
	$(CC) $(CFLAGS) -c $(IPCSRC)AppReadFromSocket.c -o $(OBJ)AppReadFromSocket.o

$(OBJ)42nos3.o         : $(SRC)42nos3.c
	$(CC) $(CFLAGS) -c $(SRC)42nos3.c -o $(OBJ)42nos3.o

$(OBJ)RbtFsw.o         : $(RBTSRC)RbtFsw.c $(RBTSRC)Rbt.h
	$(CC) $(CFLAGS) -c $(RBTSRC)RbtFsw.c -o $(OBJ)RbtFsw.o

$(OBJ)docoptkit.o     : $(KITSRC)docoptkit.c
	$(CC) $(CFLAGS) -c $(KITSRC)docoptkit.c -o $(OBJ)docoptkit.o

$(OBJ)42dsm.o       : $(SRC)42dsm.c $(INC)Ac.h $(INC)AcTypes.h $(INC)DSMTypes.h
	$(CC) $(CFLAGS) -c $(SRC)42dsm.c -o $(OBJ)42dsm.o

$(OBJ)42fssalbedo.o         : $(SRC)42fssalbedo.c
	$(CC) $(CFLAGS) -c $(SRC)42fssalbedo.c -o $(OBJ)42fssalbedo.o

########################  Miscellaneous Rules  ############################
clean :
ifeq ($(42PLATFORM),_WIN32)
	del .\Object\*.o .\$(EXENAME) .\AcApp ./DataFilter .\InOut\*.42
else ifeq ($(42PLATFORM),_WIN64)
	del .\Object\*.o .\$(EXENAME) .\AcApp ./DataFilter .\InOut\*.42
else
	rm -f $(OBJ)*.o ./$(EXENAME) ./AcApp ./DataFilter $(KITDIR)42kit.so $(INOUT)*.42 ./Standalone/*.42 ./Demo/*.42 ./Rx/*.42 ./Tx/*.42
endif

profile: CFLAGS+=-pg
profile: LFLAGS+=-pg
profile: deepthought

deploy: CFLAGS+=-O2
deploy: LFLAGS+=-O2
deploy: deepthought
