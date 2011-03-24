# Copyright (C) 2011 The Android-x86 Open Source Project

LOCAL_PATH := $(call my-dir)

# HAL module implemenation, not prelinked and stored in
# hw/<OVERLAY_HARDWARE_MODULE_ID>.<ro.product.board>.so
include $(CLEAR_VARS)
LOCAL_PRELINK_MODULE := false
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_SHARED_LIBRARIES := liblog libcutils
LOCAL_MODULE := sensors.$(TARGET_PRODUCT)
LOCAL_MODULE_TAGS := optional

ifeq ($(strip $(BOARD_USES_KBDSENSOR)),true)
LOCAL_SRC_FILES := kbdsensor.cpp
include $(BUILD_SHARED_LIBRARY)
endif
