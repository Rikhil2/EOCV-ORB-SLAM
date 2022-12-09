#include "orb-slam3/include/System.h"
#include <jni.h>

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
    s->Shutdown();
    delete s;
}

extern "C"
JNIEXPORT jfloatArray JNICALL
Java_com_scarsdalerobotics_eocv_1orb_1slam_OrbSlamJniWrapper_track(JNIEnv *env, jclass clazz,
                                                                   jlong slam_pointer,
                                                                   jlong mat_pointer, jfloat ax,
                                                                   jfloat ay, jfloat az,
                                                                   jfloat wx, jfloat wy, jfloat wz,
                                                                   jfloat t) {
    auto s = (ORB_SLAM3::System *) slam_pointer;
    auto mat = (cv::Mat *) mat_pointer;

    auto p = ORB_SLAM3::IMU::Point(ax, ay, az, wx, wy, wz, t);
    std::vector<ORB_SLAM3::IMU::Point> v = {p};

    Sophus::SE3f pose = s->TrackMonocular(*mat, t, v);

    Eigen::Vector3f trans = pose.translation(); // x, y, z
    Eigen::Vector3f rot = pose.unit_quaternion().toRotationMatrix().eulerAngles(0, 1,
                                                                                2); // roll, pitch, yaw

    // Create array
    jfloatArray result;
    result = env->NewFloatArray(6);
    if (result == nullptr) {
        return nullptr; /* out of memory error thrown */
    }

    jfloat fill[6] = {trans.x(), trans.y(), trans.z(), rot.x(), rot.y(), rot.z()};
    // move from the temp structure to the java structure
    env->SetFloatArrayRegion(result, 0, 6, fill);

    return result;
}