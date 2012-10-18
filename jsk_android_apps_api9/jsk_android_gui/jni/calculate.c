#include <jni.h>

void Java_ros_android_jskandroidgui_SensorImageViewInfo_translateBGRtoRGB(JNIEnv* env,jobject this,jintArray src,jint width,jint height){

  int i, totalPixel;
  int result = 0;

  jint* arr1 = (*env)->GetPrimitiveArrayCritical(env,src,0);

  totalPixel = width * height;

  for(i=0;i<totalPixel;i++){
    int red = (arr1[i] & 0x00FF0000) >> 16;
    int green = (arr1[i] & 0xFF00FF00);
    int blue = (arr1[i] & 0x000000FF) << 16;
    arr1[i] = ((red | blue) | green);
  }

  (*env)->ReleasePrimitiveArrayCritical(env,src,arr1,0);

}
