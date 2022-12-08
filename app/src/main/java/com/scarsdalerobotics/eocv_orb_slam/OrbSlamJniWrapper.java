package com.scarsdalerobotics.eocv_orb_slam;

public class OrbSlamJniWrapper {
    private static native long createSlam(String vocabularyFile, String settingsFile);
    private static native void releaseSlam(long slamPointer);
}
