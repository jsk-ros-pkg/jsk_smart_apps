all: test

NDK_DIR = android-ndk-1.6_r1
NDK_FILE = $(NDK_DIR)-linux-x86.zip
NDK_URL = http://dl.google.com/android/ndk/$(NDK_FILE)
OBJ = build/host-setup.sh
OBJ_DIR = BitmapFromCompressedImage
OBJ2 = Application.mk
GINGER_PATH = `rospack find android_gingerbread_mr1`

test:
	wget $(NDK_URL)
	unzip $(NDK_FILE)
	cd $(NDK_DIR) && bash $(OBJ)
	patch -d $(GINGER_PATH)/.. -p1 < setup.patch
	cd $(NDK_DIR)/apps/ && mkdir $(OBJ_DIR)
	touch $(OBJ2)
	echo APP_PROJECT_PATH :=  $(GINGER_PATH) >> $(OBJ2)
	echo APP_MODULES := calculate >> $(OBJ2)
	mv $(OBJ2) $(NDK_DIR)/apps/$(OBJ_DIR)
	cd $(NDK_DIR) && make APP=$(OBJ_DIR)
	rm -rf $(NDK_FILE)