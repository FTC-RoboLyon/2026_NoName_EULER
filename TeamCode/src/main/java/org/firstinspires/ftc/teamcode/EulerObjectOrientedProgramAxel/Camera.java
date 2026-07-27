package org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;


public class Camera {
    private static String cameraName = "Webcam 1";

    private AprilTagProcessor aprilTagProcessor;
    ColorBlobLocatorProcessor colorLocator;
    private VisionPortal visionPortal;

    public Camera (HardwareMap hmap){
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(false)
                .setDrawTagOutline(false)
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
                .setLensIntrinsics(1666.94, 1666.94, 930.463, 618.081)
                .build();

        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)
                .setRoi(ImageRegion.entireFrame())
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setBoxFitColor(0)
                .setBlurSize(5)

                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hmap.get(WebcamName.class, cameraName));
        builder.setCameraResolution(new Size(1920, 1080));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setAutoStopLiveView(false);
        builder.addProcessor(aprilTagProcessor);
        builder.addProcessor(colorLocator);

        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
        visionPortal.setProcessorEnabled(colorLocator, false);

        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
    }

    public double getDistanceMeters (int id){
        for (AprilTagDetection aprilTag : aprilTagProcessor.getDetections()){
            if (aprilTag.id == id)
                return aprilTag.ftcPose.range;
        }
        return -1.0;  // if id not detected, return a value that the camera would never return
    }

    //Et il se passe quoi si tu utilise un processor a l'arret ? Une erreur ? Un retour null ? Le renvoi des dernieres mesures ? (on aimerait eviter au moins deux des trois)
    public double getBearing (int id){
        for (AprilTagDetection aprilTag : aprilTagProcessor.getDetections()){
            if (aprilTag.id == id)
                return aprilTag.ftcPose.bearing;
        }
        return 7.0; // if id not detected, return a value that the camera would never return (the camera only reach pi radiant)
    }

    public ColorBlobLocatorProcessor.Blob getBigestBlob(){
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                50, 20000, blobs);
        ColorBlobLocatorProcessor.Util.sortByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs);
        if (!blobs.isEmpty()) {
            return blobs.get(0);
        } else
            return null;
    }

    public void setColorProcessorEnabled (boolean enabled){
        visionPortal.setProcessorEnabled(colorLocator, enabled);
    }

    public void setAprilTagProcessorEnabled (boolean enabled){
        visionPortal.setProcessorEnabled(aprilTagProcessor, enabled);
    }


    public void close()
    {
        visionPortal.close();
    }

}
