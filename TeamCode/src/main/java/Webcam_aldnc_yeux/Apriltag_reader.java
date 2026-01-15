package Webcam_aldnc_yeux;

import android.graphics.Canvas;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

public class Apriltag_reader extends AprilTagProcessor {

    public static AprilTagProcessor aprilTag ;

    public Apriltag_reader() {

         aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
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


    @Override
    public void setDecimation(float decimation) {

    }

    @Override
    public void setPoseSolver(PoseSolver poseSolver) {

    }

    @Override
    public int getPerTagAvgPoseSolveTime() {
        return 0;
    }

    @Override
    public ArrayList<AprilTagDetection> getDetections() {
        return null;
    }

    @Override
    public ArrayList<AprilTagDetection> getFreshDetections() {
        return null;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
