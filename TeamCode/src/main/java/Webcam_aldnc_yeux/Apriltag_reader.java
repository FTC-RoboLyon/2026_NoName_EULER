package Webcam_aldnc_yeux;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class Apriltag_reader {

    public static AprilTagProcessor aprilTag;
    private  VisionPortal visionPortal;
    private List<AprilTagDetection> detections = new ArrayList<>();
    private Telemetry telemetry;


    public Apriltag_reader(HardwareMap hardware) {

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
        builder.setCameraResolution(new Size(1920, 1080));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setAutoStopLiveView(false);
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(aprilTag, true);
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);




    }
    public double getTagXInPixels(int tagId) {
        AprilTagDetection tag = getAprilTagById(tagId);
        if (tag == null) return 0; // Aucun tag trouvé
        int imageWidth = 1920; // largeur de la caméra en pixels
        return tag.center.x - (imageWidth / 2.0);
    }

    // Récupère la position X normalisée entre -1 et 1, pratique pour PID
    public double getTagXNormalized(int tagId) {
        AprilTagDetection tag = getAprilTagById(tagId);
        if (tag == null) return 0; // Aucun tag trouvé
        int imageWidth = 1920;
        return (tag.center.x - (imageWidth / 2.0)) / (imageWidth / 2.0);
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

    public void telemetry(Telemetry telemetry){
        telemetry.addData("# AprilTags Detected", detections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
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
