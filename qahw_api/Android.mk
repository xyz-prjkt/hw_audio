ifeq ($(strip $(BOARD_SUPPORTS_QAHW)),true)

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libqahw
LOCAL_MODULE_TAGS := optional
LOCAL_C_INCLUDES := $(LOCAL_PATH)/inc

LOCAL_SRC_FILES := \
    src/qahw_api.cpp

ifeq ($(strip $(AUDIO_FEATURE_ENABLED_GCOV)),true)
LOCAL_CFLAGS += --coverage -fprofile-arcs -ftest-coverage
LOCAL_CPPFLAGS += --coverage -fprofile-arcs -ftest-coverage
LOCAL_STATIC_LIBRARIES += libprofile_rt
endif

LOCAL_HEADER_LIBRARIES := \
    libqahw_headers

LOCAL_SHARED_LIBRARIES := \
    liblog \
    libcutils \
    libhardware \
    libdl \
    libutils \
    libqahwwrapper

LOCAL_CFLAGS += -Wall -Werror

LOCAL_COPY_HEADERS_TO   := mm-audio/qahw_api/inc
LOCAL_COPY_HEADERS      := inc/qahw_defs.h
LOCAL_COPY_HEADERS      += inc/qahw_api.h
LOCAL_COPY_HEADERS      += inc/qahw_effect_audiosphere.h
LOCAL_COPY_HEADERS      += inc/qahw_effect_bassboost.h
LOCAL_COPY_HEADERS      += inc/qahw_effect_environmentalreverb.h
LOCAL_COPY_HEADERS      += inc/qahw_effect_equalizer.h
LOCAL_COPY_HEADERS      += inc/qahw_effect_presetreverb.h
LOCAL_COPY_HEADERS      += inc/qahw_effect_virtualizer.h
LOCAL_COPY_HEADERS      += inc/qahw_effect_visualizer.h

LOCAL_VENDOR_MODULE     := true

include $(BUILD_SHARED_LIBRARY)

include $(CLEAR_VARS)

LOCAL_MODULE := libqahwapi_headers
LOCAL_EXPORT_C_INCLUDE_DIRS := $(LOCAL_PATH)/inc
LOCAL_PROPRIETARY_MODULE := true

include $(BUILD_HEADER_LIBRARY)

#test app compilation
include $(LOCAL_PATH)/test/Android.mk

endif
