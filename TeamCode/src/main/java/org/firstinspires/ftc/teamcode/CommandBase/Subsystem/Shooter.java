package org.firstinspires.ftc.teamcode.CommandBase.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.CommandBase.robotContainer;
import org.firstinspires.ftc.teamcode.Lib.utils;

public class Shooter extends SubsystemBase {
    private DcMotorEx flywheelMotor;
    private Servo hoodServo;
    private ElapsedTime PDFTimer = new ElapsedTime();
    private robotContainer robot;

    public static final double GEAR_RATIO = 1.0;
    public static final double TICKS_PER_ROTATION = 28;

    public static final double FLYWHEEL_KP = 1.0, FLYWHEEL_KF = 1.0, FLYWHEEL_KD = 1.0; //TUNEME

    public static final double HOOD_TOLERANCE = 100.0;  //TUNEME
    public static final double FLYWHEEL_TOLERANCE = 100.0;  //TUNEME in RPM

    public static final double NEAR_POS_HOOD = 0.3, MID_POS_HOOD = 0.58, FAR_POS_HOOD = 0.45; //TUNEME between 0 and 1
    public static final double NEAR_FLYWHEEL_RPM = 1250, MID_FLYWHEEL_RPM = 1500, FAR_FLYWHEEL_RPM = 1500; //TUNEME in RPM

    private static double flywheelVeloTarget = 0.0;
    private static double hoodPosTarget = 0.0;
    private static double previousError = 0.0;
    private static double previousTime = 0.0;
    private boolean firstIteration = true;

    private enum WantedState {
        IDLE,
        NEAR_POS,
        MID_POS,
        FAR_POS,
        MANUAL
    }
    private enum SystemState {
        IDLE,
        PREPARING_SHOOT,
        READY_SHOOT

    }
    private WantedState shooterWantedState = WantedState.IDLE;
    private SystemState shooterSystemState = SystemState.IDLE;
    public void setWantedState (WantedState state) {shooterWantedState = state;}
    public void setTargets(double flywheelTarget, double HoodPos) {
        flywheelVeloTarget = flywheelTarget;
        hoodPosTarget = HoodPos;
        shooterWantedState = WantedState.MANUAL;
    }
    public SystemState getSystemState() {return shooterSystemState;}


    public Shooter(HardwareMap hmap, String shooterName, String hoodName){
        flywheelMotor = hmap.get(DcMotorEx.class, shooterName);
        hoodServo = hmap.get(Servo.class, hoodName);
        robot = new robotContainer(hmap);

        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PDFTimer.startTime();
        PDFTimer.reset();
    }


    public void stopShooter(){
        flywheelMotor.setPower(0.0);
        hoodServo.setPosition(NEAR_POS_HOOD);
    }

    @Override
    public void periodic(){

        double flywheelRPM = utils.TickPerSecondToRPM(flywheelMotor.getVelocity(), TICKS_PER_ROTATION, GEAR_RATIO);

        switch (shooterWantedState){
            case IDLE:
                shooterSystemState = SystemState.IDLE;
                break;
            case NEAR_POS:
                flywheelVeloTarget = NEAR_FLYWHEEL_RPM;
                hoodPosTarget = NEAR_POS_HOOD;
                shooterSystemState = SystemState.PREPARING_SHOOT;
                break;
            case MID_POS:
                flywheelVeloTarget = MID_FLYWHEEL_RPM;
                hoodPosTarget = MID_POS_HOOD;
                shooterSystemState = SystemState.PREPARING_SHOOT;
                break;
            case FAR_POS:
                flywheelVeloTarget = FAR_FLYWHEEL_RPM;
                hoodPosTarget = FAR_POS_HOOD;
                shooterSystemState = SystemState.PREPARING_SHOOT;
                break;
            case MANUAL:
                shooterSystemState = SystemState.PREPARING_SHOOT;
                break;
            default:
                shooterWantedState = WantedState.IDLE;
                break;
        }

        switch (shooterSystemState){
            case IDLE:
                stopShooter();
                break;
            case PREPARING_SHOOT:
                if (utils.IsInRange(flywheelRPM, flywheelVeloTarget, FLYWHEEL_TOLERANCE) && utils.IsInRange(hoodServo.getPosition(), hoodPosTarget, HOOD_TOLERANCE)){
                    shooterSystemState = SystemState.READY_SHOOT;
                    firstIteration = true;
                    break;

                } else {

                    double actualError = flywheelVeloTarget - flywheelRPM;

                    double feedForward = (FLYWHEEL_KF * flywheelVeloTarget);

                    double proportional = actualError * FLYWHEEL_KP;

                    double actualTime = PDFTimer.milliseconds();
                    if (firstIteration){
                        previousError = actualError;
                        firstIteration = false;
                    }
                    double derivative = FLYWHEEL_KD * (actualError - previousError / actualTime - previousTime);

                    double flywheelPower = proportional + derivative + feedForward;
                    flywheelMotor.setPower( utils.getVoltageCompensated(flywheelPower, robot.getVoltage()) );

                    previousError = actualError;
                    previousTime = actualTime;

                    hoodServo.setPosition(hoodPosTarget);
                }
            case READY_SHOOT:
                break;
            default:
                shooterSystemState = SystemState.IDLE;
                break;
        }
    }
}
