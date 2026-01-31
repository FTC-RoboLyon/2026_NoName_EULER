package Webcam_aldnc_yeux;

import android.annotation.SuppressLint;
import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

public class Apriltag_reader {

    public static AprilTagProcessor aprilTag ;
    private  VisionPortal visionPortal;
    private List<AprilTagDetection> detections = new ArrayList<>();
    private Telemetry telemetry;

    public Apriltag_reader(HardwareMap hardware, Telemetry telemetry) {
        this.telemetry = telemetry;

         aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                 .setLensIntrinsics(1666.94, 1666.94, 930.463, 618.081)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardware.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        builder.setCameraResolution(new Size(640, 480));
        //.setStreamFormat(VisionPortal.StreamFormat.YUY2)
        builder.setAutoStopLiveView(false);

        visionPortal = builder.build();



    }

    public List<AprilTagDetection> getdetections() {
        return detections;
    }
    public void updtade (){detections = aprilTag.getDetections();}
    public AprilTagDetection getClosestApril() {
        if (detections.isEmpty()) {
            return null;
        }
        AprilTagDetection tag = detections.get(0);
        double minDistance = tag.ftcPose.z;

        for (AprilTagDetection i : detections){
            if (i.ftcPose.z < minDistance) {
                minDistance = i.ftcPose.z;
                tag = i;
            }
        }

        return tag;
    }

    @SuppressLint("DefaultLocale")
    public void telemetry(AprilTagDetection Detection){
        if (Detection == null){return;}
        if (Detection.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", Detection.id, Detection.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", Detection.ftcPose.x, Detection.ftcPose.y, Detection.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", Detection.ftcPose.pitch, Detection.ftcPose.roll, Detection.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", Detection.ftcPose.range, Detection.ftcPose.bearing, Detection.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", Detection.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", Detection.center.x, Detection.center.y));
        }
    }

    public AprilTagDetection getAprilTagById(int wantedId) {
        for (AprilTagDetection detection : detections) {
            if (detection.id == wantedId) {
                return detection;
            }
        }
        return null;
    }

    public AprilTagDetection getBestAprilTag(int priorityId) {
        return priorityId > 0 ? getAprilTagById(priorityId) : getClosestApril();
    }

    public void stop (){
        if (visionPortal != null){
            visionPortal.close();
        }
    }
}
