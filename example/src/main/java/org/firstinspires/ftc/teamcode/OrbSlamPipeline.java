package org.firstinspires.ftc.teamcode;

import com.scarsdalerobotics.eocv_orb_slam.OrbSlamJniWrapper;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class OrbSlamPipeline extends OpenCvPipeline {
    private long nativePtr = 0;

    // TODO: Check units
    private Acceleration imuAccel = new Acceleration();
    private AngularVelocity imuOmega = new AngularVelocity();
    private final Object imuSync = new Object();

    private long startTimeMillis;

    public OrbSlamPipeline(String vocabFilePath, String settingsFilePath) {
       nativePtr = OrbSlamJniWrapper.createSlam(vocabFilePath, settingsFilePath);
       startTimeMillis = System.currentTimeMillis();
    }

    @Override
    public Mat processFrame(Mat input) {
        float ax, ay, az, wx, wy, wz;
        synchronized (imuSync) {
            ax = (float) imuAccel.xAccel;
            ay = (float) imuAccel.yAccel;
            az = (float) imuAccel.zAccel;

            wx = (float) imuOmega.xRotationRate;
            wy = (float) imuOmega.yRotationRate;
            wz = (float) imuOmega.zRotationRate;
        }

        long elapsedMillis = System.currentTimeMillis() - startTimeMillis;
        float elapsedSecs = elapsedMillis / 1000f; // TODO: Check units

        float[] pose = OrbSlamJniWrapper.track(nativePtr, input.nativeObj, ax, ay, az, wx, wy, wz, elapsedSecs);

        return input;
    }

    public void setImuData(Acceleration a, AngularVelocity w) {
       synchronized (imuSync) {
           imuAccel = a;
           imuOmega = w;
       }
    }
}
