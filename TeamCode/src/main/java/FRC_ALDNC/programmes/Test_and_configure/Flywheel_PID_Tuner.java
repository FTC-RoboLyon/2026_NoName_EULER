package FRC_ALDNC.programmes.Test_and_configure;

import static FRC_ALDNC.CONSTANT.constante_feeder.FEEDER;
import static FRC_ALDNC.CONSTANT.Constante_intake.INTAKE;
import static FRC_ALDNC.CONSTANT.Constante_shooter.SHOOTER;
import static FRC_ALDNC.CONSTANT.constante_feeder.posFeed;
import static FRC_ALDNC.CONSTANT.constante_feeder.posFeedrepos;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Flywheel PID Tuner")
public class Flywheel_PID_Tuner extends OpMode {
    public DcMotorEx flywheelMotor;
    private Servo leftdoor;

    private Servo viseur;

    private DcMotor intake;

    public static double highVelocity = 1500;
    public static double lowVelocity = 900;
    public static double P = 140.0;
    public static double I = 0;
    public static double D = 0;
    public static double F = 11.8;

    private double curTargetVelocity = highVelocity;
    private boolean motorsRunning = false;
    private ElapsedTime runtime = new ElapsedTime();
    private double loopTime = 0.0;


    private double maxError = 0;
    private double avgError = 0;
    private int sampleCount = 0;
    private double errorSum = 0;
    public static double viseur_pos;

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, SHOOTER);
        leftdoor = hardwareMap.get(Servo.class, FEEDER);
        intake = hardwareMap.get(DcMotor.class, INTAKE);
        viseur = hardwareMap.get(Servo.class, "viseur");


        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        updatePIDFCoefficients();

        telemetry.addLine("=== Flywheel PID Tuner ===");
        telemetry.addLine("Open Panels in FTC Dashboard");
        telemetry.addLine("Graphs will auto-populate!");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("X: Start/Stop Motors");
        telemetry.addLine("Y: Toggle High/Low Velocity");
        telemetry.addLine("RT: Test Rapid Fire");
        telemetry.addLine();
        telemetry.addLine("Tune PIDF in Dashboard Config");
        telemetry.update();
    }

    @Override
    public void loop() {
        viseur.setPosition(viseur_pos);

        if (gamepad1.xWasPressed()) {
            motorsRunning = !motorsRunning;
            if (motorsRunning) {
                runtime.reset();
                resetMetrics();
            }
        }

        if (gamepad1.yWasPressed()) {
            curTargetVelocity = (curTargetVelocity == highVelocity) ? lowVelocity : highVelocity;
            resetMetrics();
        }

        updatePIDFCoefficients();

        if (gamepad1.right_trigger > 0.3) {
            leftdoor.setPosition(posFeed);

        }
         else {

            leftdoor.setPosition(posFeedrepos);
        }
         if (gamepad1.right_bumper){
             intake.setPower(0.7);
         } else
             intake.setPower(0);

        if (motorsRunning) {
            flywheelMotor.setVelocity(curTargetVelocity);
        }else {
            flywheelMotor.setVelocity(0);
        }

        displayTelemetry();
    }

    private void updatePIDFCoefficients() {
        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    private void resetMetrics() {
        maxError = 0;
        avgError = 0;
        sampleCount = 0;
        errorSum = 0;
    }

    private void displayTelemetry() {
        double vel1 = flywheelMotor.getVelocity();

        double error = Math.abs(curTargetVelocity - vel1);
        double errorPercent = (error / curTargetVelocity) * 100.0;

        if (motorsRunning) {
            errorSum += error;
            sampleCount++;
            avgError = errorSum / sampleCount;
            maxError = Math.max(maxError, error);
        }


        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Motor 1 Velocity", vel1);
        telemetry.addData("Error", error);
        telemetry.addData("Error Percent", errorPercent);

        telemetry.addData("P", P);
        telemetry.addData("I", I);
        telemetry.addData("D", D);
        telemetry.addData("F", F);

        telemetry.addData("Max Error", maxError);
        telemetry.addData("Avg Error", avgError);
        telemetry.addData("Motors Running", motorsRunning);
        telemetry.addData("Runtime", runtime.seconds());
        telemetry.addData("Loop Hz", getHz());
        telemetry.addData("viseur pos", viseur_pos);

        String stability = errorPercent < 1 ? "EXCELLENT" :
                errorPercent < 2 ? "GOOD" :
                        errorPercent < 5 ? "ACCEPTABLE" : "POOR";
        telemetry.addData("Stability", stability);

        telemetry.update();
    }

    public double getHz() {
        double loop = System.nanoTime();
        double hz = 1000000000 / (loop - loopTime);
        loopTime = loop;
        return hz;
    }

    private void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
