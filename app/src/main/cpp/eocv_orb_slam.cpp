#include "orb-slam3/include/System.h"
#include <jni.h>

// Write C++ code here.
//
// Do not forget to dynamically load the C++ library into your application.
//
// For instance,
//
// In MainActivity.java:
//    static {
//       System.loadLibrary("eocv_orb_slam");
//    }
//
// Or, in MainActivity.kt:
//    companion object {
//      init {
//         System.loadLibrary("eocv_orb_slam")
//      }
//    }

std::string ConvertJString(JNIEnv *env, jstring str) {
    if (!str) std::string();

    const jsize len = env->GetStringUTFLength(str);
    const char *strChars = env->GetStringUTFChars(str, 0);

    std::string result(strChars, len);

    env->ReleaseStringUTFChars(str, strChars);

    return result;
}

extern "C"
JNIEXPORT jlong JNICALL
Java_com_scarsdalerobotics_eocv_1orb_1slam_OrbSlamJniWrapper_createSlam(JNIEnv *env, jclass clazz,
                                                                        jstring vocabulary_file,
                                                                        jstring settings_file) {
    std::string vocab = ConvertJString(env, vocabulary_file);
    std::string settings = ConvertJString(env, settings_file);

    auto s = new ORB_SLAM3::System(vocab, settings, ORB_SLAM3::System::IMU_MONOCULAR, false);

    return (long) s;
}
extern "C"
JNIEXPORT void JNICALL
Java_com_scarsdalerobotics_eocv_1orb_1slam_OrbSlamJniWrapper_releaseSlam(JNIEnv *env, jclass clazz,
                                                                         jlong slam_pointer) {

    auto s = (ORB_SLAM3::System *) slam_pointer;
    delete s;
}
