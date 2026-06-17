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

/*
- Truc plus global pour tes outputs qui veulent dire que rien n'est détécté (tes -1.0 et 7.0) la tu travailles tout seul donc tu sais ce que ça ve dire mais si tu travaillais en grp ça
  serait bien de faire les specs de la fonction et de préciser que ça correspond à ça dedans
 */
public class Camera {
    private static String cameraName = "Webcam 1";

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    public Camera (HardwareMap hmap){
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true) //-> on peut laisser true pour le dev et le debug mais je pense que en compet il faut pas oublier de le passer en false car cela doit ralentir le process
                .setDrawTagOutline(true) //-> on peut laisser true pour le dev et le debug mais je pense que en compet il faut pas oublier de le passer en false car cela doit ralentir le process
                .setDrawAxes(true) //-> on peut laisser true pour le dev et le debug mais je pense que en compet il faut pas oublier de le passer en false car cela doit ralentir le process
                .setDrawCubeProjection(true) //-> on peut laisser true pour le dev et le debug mais je pense que en compet il faut pas oublier de le passer en false car cela doit ralentir le process
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES) //->On prefere travailler dans le SI pour que ça soit mieux compréhensible par tout le monde ce qui donnerai plutot en metre et en radians
                .setLensIntrinsics(1666.94, 1666.94, 930.463, 618.081)//Juste par curiosite tu les sors d'ou c'est valeurs ?
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hmap.get(WebcamName.class, cameraName));
        builder.setCameraResolution(new Size(1920, 1080));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setAutoStopLiveView(false);
        builder.addProcessor(aprilTagProcessor); //-> pas tres important mais je pense que tu aurais pu tout mettre a la suite comme au dessus
        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
    }

    public double getDistanceInch (int id){ //pk tu utilises de inch ? (pareil c'est mieux de convertir dans le SI donc en m)
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
        return 7.0; //pk 7.0 c'est tres aleatoire
    }

    public void close() //->Tjr mieux de l'arreter a la fin du code quand c'est possible
    {
        visionPortal.close();
    }

}
