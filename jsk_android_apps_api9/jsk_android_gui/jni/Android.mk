LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := calculate
LOCAL_SRC_FILES := calculate.c

include $(BUILD_SHARED_LIBRARY)