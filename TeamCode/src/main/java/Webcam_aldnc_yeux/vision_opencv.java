package Webcam_aldnc_yeux;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class vision_opencv {

    private OpenCvCamera camera;

    public vision_opencv(HardwareMap hardwareMap) {
        // Récupérer l'ID pour l'affichage de la caméra
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );

        // Créer la webcam
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(
                        hardwareMap.get(WebcamName.class, "Webcam 1"),
                        cameraMonitorViewId
                );

        // Définir le pipeline (classe pipeline à créer)
        pipeline pipeline = new pipeline();
        camera.setPipeline(pipeline);

        // Ouvrir la caméra de façon asynchrone
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Démarrer le streaming lorsque la caméra est prête
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // Gestion simple des erreurs
                System.out.println("Erreur ouverture caméra : " + errorCode);
            }
        });
    }

    // Arrêter la caméra proprement
    public void stop() {
        if (camera != null) {
            camera.stopStreaming();
            camera.closeCameraDevice();
        }
    }
    public boolean isObjectDetected() {
        return pipeline.isObjectDetected();
    }

    public int getObjectX() {
        return pipeline.getCenterX();
    }
}
