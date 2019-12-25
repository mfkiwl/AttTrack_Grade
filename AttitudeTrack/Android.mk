LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE:= AttTrack

LOCAL_SRC_FILES:= ATProcess.cpp AttInit.cpp  AttTrack_lib.cpp Calibrate.cpp ComFunc.cpp config.cpp DataStruct.cpp ElevationEstimate.cpp kalmanfilter.cpp log.cpp Matrix.cpp  mem.c parson.cpp StaticDetect.cpp 

LOCAL_C_INCLUDES +=$(LOCAL_PATH)

include $(BUILD_SHARED_LIBRARY)

