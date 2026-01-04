package Webcam_aldnc_yeux;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTag_Reader {
    private final VisionPortal visionPortal;
    private static AprilTagProcessor aprilTag = null;

    public AprilTag_Reader(HardwareMap hardwareMap) {

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    public static List<AprilTagDetection> getdetections() {
        return aprilTag.getDetections();
    }
    public AprilTagDetection getClosestApril() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
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

    public AprilTagDetection getAprilTagById(int wantedId) {
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection.id == wantedId) {
                return detection;
            }
        }
        return null;
    }

    public AprilTagDetection getBestAprilTag(Integer priorityId) {
        return  priorityId != null ? getAprilTagById(priorityId) : getClosestApril();
    }
}
