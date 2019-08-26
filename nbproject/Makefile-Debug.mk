#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=MinGW-Windows
CND_DLIB_EXT=dll
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/main.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=-L/F/opencv/build/x86/MSYS/lib /F/opencv/build/x86/TBB/lib/libopencv_calib3d247.dll.a /F/opencv/build/x86/TBB/lib/libopencv_calib3d_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_contrib247.dll.a /F/opencv/build/x86/TBB/lib/libopencv_contrib_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_core247.dll.a /F/opencv/build/x86/TBB/lib/libopencv_core_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_features2d247.dll.a /F/opencv/build/x86/TBB/lib/libopencv_features2d_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_flann247.dll.a /F/opencv/build/x86/TBB/lib/libopencv_flann_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_gpu247.dll.a /F/opencv/build/x86/TBB/lib/libopencv_gpu_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_haartraining_engine.a /F/opencv/build/x86/TBB/lib/libopencv_highgui247.dll.a /F/opencv/build/x86/TBB/lib/libopencv_highgui_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_imgproc247.dll.a /F/opencv/build/x86/TBB/lib/libopencv_imgproc_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_legacy247.dll.a /F/opencv/build/x86/TBB/lib/libopencv_legacy_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_ml247.dll.a /F/opencv/build/x86/TBB/lib/libopencv_ml_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_nonfree247.dll.a /F/opencv/build/x86/TBB/lib/libopencv_nonfree_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_objdetect247.dll.a /F/opencv/build/x86/TBB/lib/libopencv_objdetect_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_ocl247.dll.a /F/opencv/build/x86/TBB/lib/libopencv_ocl_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_perf_calib3d_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_perf_core_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_perf_features2d_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_perf_gpu_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_perf_highgui_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_perf_imgproc_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_perf_nonfree_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_perf_objdetect_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_perf_ocl_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_perf_photo_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_perf_stitching_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_perf_superres_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_perf_video_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_photo247.dll.a /F/opencv/build/x86/TBB/lib/libopencv_photo_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_stitching247.dll.a /F/opencv/build/x86/TBB/lib/libopencv_stitching_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_superres247.dll.a /F/opencv/build/x86/TBB/lib/libopencv_superres_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_test_calib3d_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_test_contrib_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_test_core_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_test_features2d_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_test_flann_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_test_gpu_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_test_highgui_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_test_imgproc_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_test_legacy_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_test_ml_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_test_nonfree_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_test_objdetect_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_test_ocl_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_test_photo_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_test_stitching_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_test_superres_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_test_video_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_ts247.a /F/opencv/build/x86/TBB/lib/libopencv_ts_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_video247.dll.a /F/opencv/build/x86/TBB/lib/libopencv_video_pch_dephelp.a /F/opencv/build/x86/TBB/lib/libopencv_videostab247.dll.a /F/opencv/build/x86/TBB/lib/libopencv_videostab_pch_dephelp.a

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_calib3d247.dll.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_calib3d_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_contrib247.dll.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_contrib_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_core247.dll.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_core_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_features2d247.dll.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_features2d_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_flann247.dll.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_flann_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_gpu247.dll.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_gpu_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_haartraining_engine.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_highgui247.dll.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_highgui_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_imgproc247.dll.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_imgproc_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_legacy247.dll.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_legacy_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_ml247.dll.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_ml_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_nonfree247.dll.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_nonfree_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_objdetect247.dll.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_objdetect_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_ocl247.dll.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_ocl_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_perf_calib3d_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_perf_core_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_perf_features2d_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_perf_gpu_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_perf_highgui_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_perf_imgproc_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_perf_nonfree_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_perf_objdetect_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_perf_ocl_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_perf_photo_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_perf_stitching_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_perf_superres_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_perf_video_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_photo247.dll.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_photo_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_stitching247.dll.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_stitching_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_superres247.dll.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_superres_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_test_calib3d_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_test_contrib_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_test_core_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_test_features2d_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_test_flann_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_test_gpu_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_test_highgui_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_test_imgproc_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_test_legacy_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_test_ml_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_test_nonfree_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_test_objdetect_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_test_ocl_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_test_photo_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_test_stitching_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_test_superres_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_test_video_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_ts247.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_ts_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_video247.dll.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_video_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_videostab247.dll.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: /F/opencv/build/x86/TBB/lib/libopencv_videostab_pch_dephelp.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim ${OBJECTFILES} ${LDLIBSOPTIONS}

${OBJECTDIR}/main.o: main.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/F/opencv/build/include -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/main.o main.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/stereosim.exe

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
