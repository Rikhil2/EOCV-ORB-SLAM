package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.scarsdalerobotics.eocv_orb_slam.OrbSlamJniWrapper;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class OrbSlamPipeline extends OpenCvPipeline {
    private long nativePtr = 0;

    // TODO: Check units
    private Acceleration imuAccel = new Acceleration();
    private AngularVelocity imuOmega = new AngularVelocity();
    private final Object imuSync = new Object();

    private final long startTimeNanos;

    private Position position = new Position();
    private Orientation orientation = new Orientation();
    private final Object outSync = new Object();

    public OrbSlamPipeline(String vocabFilePath, String settingsFilePath) {
        nativePtr = OrbSlamJniWrapper.createSlam(vocabFilePath, settingsFilePath);
        startTimeNanos = System.nanoTime();
    }

    @Override
    protected void finalize() {
        if (nativePtr != 0) {
            OrbSlamJniWrapper.releaseSlam(nativePtr);
            nativePtr = 0;
        } else {
            System.out.println("ORB-SLAM3: No detector to release");
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        float ax, ay, az, wx, wy, wz;
        synchronized (imuSync) {
            ax = (float) imuAccel.xAccel;
            ay = (float) imuAccel.yAccel;
            az = (float) imuAccel.zAccel;

            wx = imuOmega.xRotationRate;
            wy = imuOmega.yRotationRate;
            wz = imuOmega.zRotationRate;
        }

        long elapsedNanos = System.nanoTime() - startTimeNanos;
        float elapsedSecs = elapsedNanos / 1e9f; // TODO: Check units

        float[] pose = OrbSlamJniWrapper.track(nativePtr, input.nativeObj, ax, ay, az, wx, wy, wz, elapsedSecs);

        Position p = new Position(
                DistanceUnit.MM,
                pose[0], pose[1], pose[2],
                elapsedNanos); // TODO: Check units
        Orientation o = new Orientation(
                AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,
                pose[3], pose[4], pose[5],
                elapsedNanos); // TODO: Check units

        synchronized (outSync) {
            position = p;
            orientation = o;
        }

        return input;
    }

    // TODO: Take multiple IMU datas (I think that helps ORB SLAM?)
    public void updateImuData(Acceleration a, AngularVelocity w) {
        synchronized (imuSync) {
            imuAccel = a.toUnit(DistanceUnit.MM);
            imuOmega = w.toAngleUnit(AngleUnit.DEGREES);
        }
    }

    public Pair<Position, Orientation> getPoseEstimate() {
        synchronized (outSync) {
            return new Pair<>(position, orientation);
        }
    }
}
