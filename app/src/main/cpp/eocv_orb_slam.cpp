#include "orb-slam3/include/System.h"

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

void hello_world() {
    ORB_SLAM3::System s("", "", ORB_SLAM3::System::IMU_MONOCULAR, false);
}