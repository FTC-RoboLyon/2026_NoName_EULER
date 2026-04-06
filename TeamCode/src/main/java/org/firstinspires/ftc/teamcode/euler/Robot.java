package org.firstinspires.ftc.teamcode.euler;

import static org.firstinspires.ftc.teamcode.euler.Constant.FEEDER_SERVO;
import static org.firstinspires.ftc.teamcode.euler.Constant.INTAKE_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.PATHER_SERVO;
import static org.firstinspires.ftc.teamcode.euler.Constant.RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.SHOOTER_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.VISEUR_SERVO;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.euler.driver.Driver;
import org.firstinspires.ftc.teamcode.euler.feeder.Feeder;
import org.firstinspires.ftc.teamcode.euler.intake.Intake;
import org.firstinspires.ftc.teamcode.euler.pather.Pather;
import org.firstinspires.ftc.teamcode.euler.shooter.Shooter;
import org.firstinspires.ftc.teamcode.euler.viseur.Viseur;

/**
 * Classe Robot englobant tous les sous-systèmes.
 * Centralise l'initialisation et la mise à jour périodique du hardware.
 */
public class Robot {
    private final Driver driver;
    private final Intake intake;
    private final Shooter shooter;
    private final Viseur viseur;
    private final Feeder feeder;
    private final Pather pather;

    /**
     * Initialise tous les sous-systèmes du robot.
     *
     * @param hardwareMap Le HardwareMap fourni par l'OpMode.
     */
    public Robot(HardwareMap hardwareMap) {
        // Initialisation des composants individuels
        this.driver = new Driver(
                hardwareMap.get(DcMotor.class, LEFT_MOTOR),
                hardwareMap.get(DcMotor.class, RIGHT_MOTOR)
        );

        this.intake = new Intake(
                hardwareMap.get(DcMotor.class, INTAKE_MOTOR)
        );

        // Récupération du premier capteur de tension disponible pour le shooter
        VoltageSensor batteryVoltage = hardwareMap.voltageSensor.iterator().next();
        this.shooter = new Shooter(
                hardwareMap.get(DcMotor.class, SHOOTER_MOTOR),
                batteryVoltage
        );

        this.viseur = new Viseur(
                hardwareMap.get(Servo.class, VISEUR_SERVO)
        );

        this.feeder = new Feeder(
                hardwareMap.get(Servo.class, FEEDER_SERVO)
        );

        this.pather = new Pather(
                hardwareMap.get(CRServo.class, PATHER_SERVO)
        );
    }

    /**
     * Met à jour tous les sous-systèmes du robot.
     * Doit être appelée à chaque itération de la boucle de l'OpMode.
     */
    public void update() {
        driver.update();
        intake.update();
        shooter.update();
        viseur.update();
        feeder.update();
        pather.update();
    }

    public Driver getDriver() {
        return driver;
    }

    public Intake getIntake() {
        return intake;
    }

    public Shooter getShooter() {
        return shooter;
    }

    public Viseur getViseur() {
        return viseur;
    }

    public Feeder getFeeder() {
        return feeder;
    }

    public Pather getPather() {
        return pather;
    }
}
