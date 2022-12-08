package com.scarsdalerobotics.eocv_orb_slam;

public class OrbSlamJniWrapper {
    static {
        System.loadLibrary("eocv_orb_slam");
    }

    // TODO: Optimize by passing in float array to fill
    private static native float[] track(long slamPointer, float ax, float ay, float az, float wx, float wy, float wz, float t);

    private static native long createSlam(String vocabularyFile, String settingsFile);

    private static native void releaseSlam(long slamPointer);
}
