package org.firstinspires.ftc.teamcode.euler.driver;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.euler.RobotTelemetry;
import org.firstinspires.ftc.teamcode.euler.TelemetryAware;
import org.firstinspires.ftc.teamcode.euler.UpdateAware;

/**
 * Sous-système gérant le déplacement du robot (Tank Drive).
 * Utilise une architecture avec séparation de l'intention et de l'exécution.
 */
public class Driver implements UpdateAware, TelemetryAware {

    private final DcMotor leftMotor;
    private final DcMotor rightMotor;

    private double targetLeftPower = 0;
    private double targetRightPower = 0;
    private RunMode parkMode = RunMode.NORMAL;

    /**
     * Initialise les moteurs du châssis.
     *
     * @param leftMotor1  Le moteur gauche du robot.
     * @param rightMotor1 Le moteur droit du robot.
     */
    public Driver(DcMotor leftMotor1, DcMotor rightMotor1) {
        this.leftMotor = leftMotor1;
        this.rightMotor = rightMotor1;

        this.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Enregistre l'intention de pilotage pour les moteurs.
     * L'application réelle de la puissance se fait lors de l'appel à {@link #update()}.
     *
     * @param left  Puissance cible pour le moteur gauche (entre -1.0 et 1.0).
     * @param right Puissance cible pour le moteur droit (entre -1.0 et 1.0).
     */
    public void drive(double left, double right) {
        this.targetLeftPower = left + right;
        this.targetRightPower = left - right;
    }

    public void toggleParkMode() {
        if (this.parkMode == RunMode.NORMAL) {
            parkMode = RunMode.PARK;
        } else {
            parkMode = RunMode.NORMAL;
        }
    }

    /**
     * Applique les puissances cibles aux moteurs physiques.
     * Doit être appelée à chaque itération.
     */
    @Override
    public void update() {
        leftMotor.setPower(targetLeftPower * parkMode.getCoef());
        rightMotor.setPower(targetRightPower * parkMode.getCoef());
    }

    @Override
    public RobotTelemetry getTelemetry() {
        return new RobotTelemetry("Chassis", "State: " + getState() + "park: " + parkMode);
    }

    /**
     * Retourne l'état réel du châssis basé sur la puissance effective des moteurs.
     *
     * @return L'état physique actuel (MOVING ou IDLE).
     */
    public DriverState getState() {
        if (leftMotor.getPower() == 0 && rightMotor.getPower() == 0) {
            return DriverState.IDLE;
        } else {
            return DriverState.MOVING;
        }
    }
}
