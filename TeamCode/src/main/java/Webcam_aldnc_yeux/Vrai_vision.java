package Webcam_aldnc_yeux;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

public class Vrai_vision implements VisionProcessor {

    Mat hsv = new Mat();
    Mat mask = new Mat();
    private static volatile boolean objectDetected = false;
    private static volatile int centerX = -1;
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

        Scalar low = new Scalar(0, 100, 100);
        Scalar high = new Scalar(10, 255, 255);

        Moments moments = Imgproc.moments(mask);
        double area = moments.get_m00();

        if (area > 500) { // seuil à ajuster
            objectDetected = true;
            centerX = (int) (moments.get_m10() / area);
        } else {
            objectDetected = false;
            centerX = -1;
        }

        return mask;
    }
    public static boolean isObjectDetected() {
        return objectDetected;
    }

    public static int getCenterX() {
        return centerX;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }


}
