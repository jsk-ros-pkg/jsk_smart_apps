all:  android-ndk

NDK_DIR		= android-ndk-1.6_r1
TARBALL		= $(NDK_DIR)-linux-x86.zip
TARBALL_URL	= http://dl.google.com/android/ndk/$(TARBALL)
SOURCE_DIR	= build/$(NDK_DIR)
MD5SUM_FILE	= $(TARBALL).md5sum
UNPACK_CMD	= unzip
OBJ_DIR		= BitmapFromCompressedImage
APP_MAKE	= $(SOURCE_DIR)/apps/$(OBJ_DIR)/Application.mk
GINGER_PATH	= `rospack find android_gingerbread_mr1`
include $(shell rospack find mk)/download_unpack_build.mk

android-ndk: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && bash build/host-setup.sh
	patch -N -d $(GINGER_PATH)/.. -p1 < setup.patch || true
	mkdir -p $(SOURCE_DIR)/apps/$(OBJ_DIR)
	echo "APP_PROJECT_PATH :=  $(GINGER_PATH)\nAPP_MODULES := calculate\n" > $(APP_MAKE)
	cd $(SOURCE_DIR) && make APP=$(OBJ_DIR)

clean:
	-rm -rf $(SOURCE_DIR)
wipe: clean
	-rm -rf build $(TARBALL)
