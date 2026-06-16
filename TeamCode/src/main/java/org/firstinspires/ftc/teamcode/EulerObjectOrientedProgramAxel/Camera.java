package org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Camera {
    private static String cameraName = "Webcam 1";

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    public Camera (HardwareMap hmap){
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setLensIntrinsics(1666.94, 1666.94, 930.463, 618.081)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hmap.get(WebcamName.class, cameraName));
        builder.setCameraResolution(new Size(1920, 1080));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setAutoStopLiveView(false);
        builder.addProcessor(aprilTagProcessor);
        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
    }

    public double getDistanceInch (int id){
        for (AprilTagDetection aprilTag : aprilTagProcessor.getDetections()){
            if (aprilTag.id == id)
                return aprilTag.ftcPose.range;
        }
        return -1.0;
    }

    public double getDistanceCM (int id){
        for (AprilTagDetection aprilTag : aprilTagProcessor.getDetections()){
            if (aprilTag.id == id)
                return aprilTag.ftcPose.range * 2.54;
        }
        return -1.0;
    }

    public double getBearing (int id){
        for (AprilTagDetection aprilTag : aprilTagProcessor.getDetections()){
            if (aprilTag.id == id)
                return Math.toRadians(aprilTag.ftcPose.bearing);
        }
        return 7.0;
    }

}
