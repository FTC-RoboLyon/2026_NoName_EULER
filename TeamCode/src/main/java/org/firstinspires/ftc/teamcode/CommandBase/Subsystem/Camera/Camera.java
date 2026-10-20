package org.firstinspires.ftc.teamcode.CommandBase.Subsystem.Camera;

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

    public final double CAMERA_FX = 1666.94, CAMERA_FY = 1666.94, CAMERA_CX = 930.463, CAMERA_CY = 618.081;

    private int targetId;

    public Camera(HardwareMap hmap){

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(false)
                .setDrawTagOutline(false)
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
                .setLensIntrinsics(CAMERA_FX, CAMERA_FY, CAMERA_CX, CAMERA_CY)
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
    public void setTargetID(int id){
        targetId = id;
    }
    public int getTargetId() {
        return targetId;
    }
    public boolean targetIDIsDetected(){
        for (AprilTagDetection aprilTag : aprilTagProcessor.getDetections()){
            if (aprilTag.id == targetId)
                return true;
        }
        return false;
    }

    public double getDistanceMeters (){
        for (AprilTagDetection aprilTag : aprilTagProcessor.getDetections()){
            if (aprilTag.id == targetId)
                return aprilTag.ftcPose.range;
        }
        return -1.0;  // if id not detected, return a value that the camera would never return
    }

    public double getBearing(){
        for (AprilTagDetection aprilTag : aprilTagProcessor.getDetections()){
            if (aprilTag.id == targetId)
                return aprilTag.ftcPose.bearing;
        }
        return 7.0; // if id not detected, return a value that the camera would never return (the camera only reach pi radiant)
    }

    public void close()
    {
        visionPortal.close();
    }

}
