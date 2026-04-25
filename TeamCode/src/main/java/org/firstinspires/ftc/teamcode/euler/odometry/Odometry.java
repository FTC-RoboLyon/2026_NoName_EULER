package org.firstinspires.ftc.teamcode.euler.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.euler.RobotTelemetry;
import org.firstinspires.ftc.teamcode.euler.TelemetryAware;
import org.firstinspires.ftc.teamcode.euler.UpdateAware;
import org.firstinspires.ftc.teamcode.euler.compass.Compass;

import java.util.Locale;

/**
 * Système d'odométrie basé sur deux roues mortes (encoders) parallèles
 * et sur la boussole (IMU) pour le cap.
 */
public class Odometry implements UpdateAware, TelemetryAware {
    // Constantes de conversion (à ajuster selon le matériel)
    // Exemple : Roue de 50mm (deadwheel), 90mm (traction) de diamètre, 28 ticks par tour (encoder externe 28192)

    public static double TICKS_PER_MM = 28192 / (50.0 * Math.PI);
    private final DcMotor leftEncoder;
    private final DcMotor rightEncoder;
    private final Compass compass;

    // Positions précédentes pour le calcul des deltas
    private double lastLeftPos = 0;
    private double lastRightPos = 0;

    // Position du robot sur le terrain (en mm et radians)
    private double x = 0;
    private double y = 0;
    private double heading = 0;

    public Odometry(DcMotor leftEncoder, DcMotor rightEncoder, Compass compass) {
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.compass = compass;
    }

    /**
     * Réinitialise la position du robot.
     */
    public void reset(Pose2D pose2D) {
        this.x = pose2D.getX(DistanceUnit.MM);
        this.y = pose2D.getY(DistanceUnit.MM);
        this.heading = pose2D.getHeading(AngleUnit.RADIANS);

        this.lastLeftPos = leftEncoder.getCurrentPosition();
        this.lastRightPos = rightEncoder.getCurrentPosition();
    }

    @Override
    public void update() {
        double currentLeft = leftEncoder.getCurrentPosition();
        double currentRight = rightEncoder.getCurrentPosition();
        double currentHeading = compass.getHeading(AngleUnit.RADIANS);

        // Calcul des déplacements des encodeurs
        double deltaLeft = (currentLeft - lastLeftPos) / TICKS_PER_MM;
        double deltaRight = (currentRight - lastRightPos) / TICKS_PER_MM;

        // Distance moyenne parcourue (puisque les roues sont parallèles)
        double distance = (deltaLeft + deltaRight) / 2.0;

        // Mise à jour de la position globale
        // On utilise le cap moyen pour une meilleure précision sur l'arc
        double avgHeading = heading + (currentHeading - heading) / 2.0;

        x = x + (distance * Math.cos(avgHeading));
        y = y + (distance * Math.sin(avgHeading));

        // Mise à jour des états
        heading = currentHeading;
        lastLeftPos = currentLeft;
        lastRightPos = currentRight;
    }

    @Override
    public RobotTelemetry getTelemetry() {
        Pose2D position = getPose();
        return new RobotTelemetry("Odometry",
                String.format(Locale.FRENCH, "X: %.1f mm, Y: %.1f mm, H: %.1f°, l: %d, r: %d", position.getX(DistanceUnit.MM), position.getY(DistanceUnit.MM), position.getHeading(AngleUnit.DEGREES), leftEncoder.getCurrentPosition(), rightEncoder.getCurrentPosition()));
    }

    /**
     * Retourne la position sur le terrain
     * <pre>
     *           ^  +X (Avant)
     *           |
     *    +Y <── O ──> -Y
     * (Gauche)  | (Droite)
     *           |
     *           v  -X (Arrière)
     * </pre>
     *
     * @return position sur le terrain
     */
    public Pose2D getPose() {
        return new Pose2D(DistanceUnit.MM, x, y, AngleUnit.RADIANS, heading);
    }

}
