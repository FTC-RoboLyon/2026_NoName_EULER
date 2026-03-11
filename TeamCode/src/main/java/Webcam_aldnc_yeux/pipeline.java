package Webcam_aldnc_yeux;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

public class pipeline extends OpenCvPipeline{
    Mat hsv = new Mat();
    Mat mask = new Mat();
    private static volatile boolean objectDetected = false;
    private static volatile int centerX = -1;
    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Scalar low = new Scalar(0, 100, 100);
        Scalar high = new Scalar(10, 255, 255);

        Core.inRange(hsv, low, high, mask);



        // Calcul du centre de l'objet
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
}
