package FRC_ALDNC.SubSystem;

import android.util.Size;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
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

public class Camera_subsystem extends SubsystemBase {
    public static AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detections = new ArrayList<>();
    public AprilTagDetection actual_detection;
    int wanted_id;
    public enum Camera_mode{
        Lock_in,
        Closest_is_the_best,
        Etre_indécis
    }
    public Camera_mode cameraMode = Camera_mode.Lock_in;
    public Telemetry telemetry;
    public FtcDashboard dashboard;
    public Camera_subsystem(HardwareMap hardware, int wanted_id, Telemetry telemetry){
        this(hardware, wanted_id, Camera_mode.Lock_in,telemetry);
    }
    public Camera_subsystem(HardwareMap hardware, int wanted_id, Camera_mode cameraMode1, Telemetry telemetry) {
        dashboard = FtcDashboard.getInstance();

        this.wanted_id = wanted_id;
        this.telemetry = telemetry;
        cameraMode = cameraMode1;
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
    public void setCameraMode(Camera_mode camera_mode){cameraMode = camera_mode;}
    public double getBearing() {
        if (actual_detection == null) return 0; // Aucun tag trouvé
        int imageWidth = 1920;
        return (actual_detection.ftcPose.bearing);
    }
    public Camera_mode getCameraMode(){return cameraMode;}
    public double getTagXInPixels() {
        if (actual_detection == null) return 0; // Aucun tag trouvé
        int imageWidth = 1920; // largeur de la caméra en pixels
        return actual_detection.center.x - (imageWidth / 2.0);
    }
    public double getTagXNormalized() {
        if (actual_detection == null) return 0; // Aucun tag trouvé
        int imageWidth = 1920;
        return (actual_detection.center.x - (imageWidth / 2.0)) / (imageWidth / 2.0);
    }
    public void telemetry(){
        telemetry.addData("# AprilTags Detected", detections.size());
        TelemetryPacket mon_ptit_truc = new TelemetryPacket();
        mon_ptit_truc.put("# AprilTags Detected", detections.size());


        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                mon_ptit_truc.put("distance to goal ", detection.ftcPose.y);

                ;
                dashboard.sendTelemetryPacket(mon_ptit_truc);
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        telemetry.update();
    }
    public void define_wanted_id(int wantedId){wanted_id = wantedId;}

    // Récupère la position X normalisée entre -1 et 1, pratique pour PID


    public List<AprilTagDetection> getdetections() {
        return detections;
    }
    public void updtade (){detections = aprilTag.getDetections();}
    public AprilTagDetection getClosestApril() {
        if (detections.isEmpty()) {
            return null;
        }
        AprilTagDetection tag = detections.get(0);
        double minDistance = tag.ftcPose.x;

        for (AprilTagDetection i : detections){
            if (i.ftcPose.z < minDistance) {
                minDistance = i.ftcPose.x;
                tag = i;
            }
        }

        return tag;
    }
    public List<AprilTagDetection> getDetections (){return detections;}
    public AprilTagDetection getAprilTagById() {
        for (AprilTagDetection detection : detections) {
            if (detection.id == wanted_id) {
                return detection;
            }
        }
        return null;
    }
    public AprilTagDetection getActual_detection(){

        return actual_detection;}


    public void stop (){
        if (visionPortal != null){
            visionPortal.close();
        }
    }
    public boolean wanted_id_is_detected (){
        for (AprilTagDetection detection : detections){
            if (detection.id == wanted_id)
                return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        updtade();
        switch (cameraMode){
            case Lock_in:
                actual_detection = getAprilTagById();
                break;
            case Closest_is_the_best:
                actual_detection = getClosestApril();
                break;
            case Etre_indécis:
                actual_detection = wanted_id_is_detected() ? getAprilTagById() : getClosestApril();
                break;
        }
        telemetry();
    }
}
