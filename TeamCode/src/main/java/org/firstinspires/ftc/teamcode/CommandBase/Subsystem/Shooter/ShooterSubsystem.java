package org.firstinspires.ftc.teamcode.CommandBase.Subsystem.Shooter;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.CommandBase.robotContainer;
import org.firstinspires.ftc.teamcode.Lib.LyonLib.control.ControlMode;
import org.firstinspires.ftc.teamcode.Lib.utils;

import java.util.function.BooleanSupplier;

public class ShooterSubsystem extends SubsystemBase {
    private DcMotorEx flywheelMotor;
    private Servo hoodServo;
    private ElapsedTime PDFTimer = new ElapsedTime();
    private robotContainer robot;
    private BooleanSupplier plusSpeedSupplier = ()-> false;
    private BooleanSupplier minusSpeedSupplier = ()-> false;
    private BooleanSupplier increaseIncrementationSupplier = ()-> false;
    private BooleanSupplier decreaseIncrementationSupplier = ()-> false;
    private double speedIncrementation;

    public static final double GEAR_RATIO = 1.0;
    public static final double TICKS_PER_ROTATION = 28;

    public static final double FLYWHEEL_KP = 1.0, FLYWHEEL_KF = 1.0, FLYWHEEL_KD = 1.0; //TUNEME

    public static final double FLYWHEEL_TOLERANCE = 100.0;  //TUNEME in RPM

    public static final double NEAR_POS_HOOD = 0.3, MID_POS_HOOD = 0.58, FAR_POS_HOOD = 0.45; //TUNEME between 0 and 1, the percentage of the angle your servo can turn between the minimum position and the maximum position
    public static final double NEAR_FLYWHEEL_RPM = 1250, MID_FLYWHEEL_RPM = 1500, FAR_FLYWHEEL_RPM = 1500; //TUNEME in RPM

    private static double flywheelActualSpeedRPM = 0.0;
    private static double flywheelSpeedTarget = 0.0;
    private static double hoodPosTarget = 0.0;
    private static double previousError = 0.0;
    private static double previousTime = 0.0;
    private boolean firstIteration = true;
    private boolean ballHasPassed;

    public enum WantedState {
        STAND_BY,
        SHOOT_NEAR,
        SHOOT_MID,
        SHOOT_FAR,
        SHOOT_AUTO
    }
    public enum SystemState {
        IDLE,
        PREPARING_SHOOT_NEAR,
        PREPARING_SHOOT_MID,
        PREPARING_SHOOT_FAR,
        PREPARING_SHOOT_AUTO,

        READY_TO_SHOOT_AUTO,
        READY_TO_SHOOT_NEAR,
        READY_TO_SHOOT_MID,
        READY_TO_SHOOT_FAR
    }
    private WantedState wantedState = WantedState.STAND_BY;
    private SystemState systemState = SystemState.IDLE;
    private ControlMode shooterControlMode = ControlMode.DISABLED;
    public void setWantedState (WantedState state) {
        wantedState = state;
    }
    public void setShooterControlMode (ControlMode controlMode){shooterControlMode = controlMode;}


    public SystemState getSystemState() {return systemState;}
    public ControlMode getShooterControlMode(){return shooterControlMode;}


    public ShooterSubsystem(HardwareMap hmap, String shooterName, String hoodName, robotContainer robotContainer){
        flywheelMotor = hmap.get(DcMotorEx.class, shooterName);
        hoodServo = hmap.get(Servo.class, hoodName);
        robot = robotContainer;

        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PDFTimer.startTime();
        PDFTimer.reset();

    }

    public void defineSupplier(BooleanSupplier plusSpeedSupplier,
                               BooleanSupplier minusSpeedSupplier,

                               BooleanSupplier plusIncrementationSupplier,
                               BooleanSupplier minusIncrementationSupplier)
    {

        this.plusSpeedSupplier = plusSpeedSupplier;
        this.minusSpeedSupplier = minusSpeedSupplier;

        this.increaseIncrementationSupplier = plusIncrementationSupplier;
        this.decreaseIncrementationSupplier = minusIncrementationSupplier;
    }


    public void stopShooter(){
        flywheelMotor.setPower(0.0);
        hoodServo.setPosition(MID_POS_HOOD);
    }


    @Override
    public void periodic(){

        switch (shooterControlMode)
        {
            case DISABLED:
                break;
            case MANUAL_VELOCITY:
                if (plusSpeedSupplier.getAsBoolean())
                    flywheelSpeedTarget += speedIncrementation;
                if (minusSpeedSupplier.getAsBoolean())
                    flywheelSpeedTarget -= speedIncrementation;

                if (increaseIncrementationSupplier.getAsBoolean())
                    speedIncrementation *= 10;
                if(decreaseIncrementationSupplier.getAsBoolean())
                    speedIncrementation /= 10;

                applyFlywheelSpeedWithPIDF();
                hoodServo.setPosition(hoodPosTarget);

                robot.getTelemetry().addData("Incrementation", speedIncrementation);
                break;

            case VELOCITY_AUTO_PIDF:

                flywheelActualSpeedRPM = utils.TickPerSecondToRPM(flywheelMotor.getVelocity(), TICKS_PER_ROTATION, GEAR_RATIO);

                RunStateMachine();

                switch (systemState) {
                    case IDLE:
                        break;
                    case PREPARING_SHOOT_AUTO:
                    case READY_TO_SHOOT_AUTO:
                        updateTargetsDistance();
                        applyFlywheelSpeedWithPIDF();
                        hoodServo.setPosition(hoodPosTarget);
                        break;

                    case PREPARING_SHOOT_NEAR:
                    case PREPARING_SHOOT_MID:
                    case PREPARING_SHOOT_FAR:
                    case READY_TO_SHOOT_NEAR:
                    case READY_TO_SHOOT_MID:
                    case READY_TO_SHOOT_FAR:
                        applyFlywheelSpeedWithPIDF();
                        hoodServo.setPosition(hoodPosTarget);
                        break;

                    default:
                        systemState = SystemState.IDLE;
                        break;
                }
                break;
            default:
                robot.getTelemetry().addLine("Please enter a valid Shooter ControlMode");
                break;
        }

    }

    private void RunStateMachine()
    {
        switch (wantedState)
        {
            case STAND_BY:
                systemState = SystemState.IDLE;
                break;

            case SHOOT_NEAR:
                flywheelSpeedTarget = NEAR_FLYWHEEL_RPM;
                hoodPosTarget = NEAR_POS_HOOD;
                if (systemState != SystemState.READY_TO_SHOOT_NEAR)
                {
                    if (systemState != SystemState.PREPARING_SHOOT_NEAR)
                        firstIteration = true;
                    systemState = SystemState.PREPARING_SHOOT_NEAR;
                }
                break;

            case SHOOT_MID:
                flywheelSpeedTarget = MID_FLYWHEEL_RPM;
                hoodPosTarget = MID_POS_HOOD;
                if (systemState != SystemState.READY_TO_SHOOT_MID)
                {
                    if (systemState != SystemState.PREPARING_SHOOT_MID)
                        firstIteration = true;
                    systemState = SystemState.PREPARING_SHOOT_MID;
                }
                break;

            case SHOOT_FAR:
                flywheelSpeedTarget = FAR_FLYWHEEL_RPM;
                hoodPosTarget = FAR_POS_HOOD;
                if (systemState != SystemState.READY_TO_SHOOT_FAR)
                {
                    if (systemState != SystemState.PREPARING_SHOOT_FAR)
                        firstIteration = true;
                    systemState = SystemState.PREPARING_SHOOT_FAR;
                }
                break;

            case SHOOT_AUTO:
                updateTargetsDistance();
                if (systemState != SystemState.READY_TO_SHOOT_AUTO)
                {
                    if (systemState != SystemState.PREPARING_SHOOT_AUTO)
                        firstIteration = true;
                    systemState = SystemState.PREPARING_SHOOT_AUTO;
                }
                break;

            default:
                wantedState = WantedState.STAND_BY;
                robot.getTelemetry().addLine("Please enter a valid shooter wanted state");
                break;

        }

        switch (systemState){
            case IDLE:
                break;

            case PREPARING_SHOOT_NEAR:
                if (utils.IsInRange(flywheelActualSpeedRPM, flywheelSpeedTarget, FLYWHEEL_TOLERANCE)) {
                    systemState = SystemState.READY_TO_SHOOT_NEAR;
                }
                break;

            case PREPARING_SHOOT_MID:
                if (utils.IsInRange(flywheelActualSpeedRPM, flywheelSpeedTarget, FLYWHEEL_TOLERANCE)) {
                    systemState = SystemState.READY_TO_SHOOT_MID;
                }
                break;

            case PREPARING_SHOOT_FAR:
                if (utils.IsInRange(flywheelActualSpeedRPM, flywheelSpeedTarget, FLYWHEEL_TOLERANCE)) {
                    systemState = SystemState.READY_TO_SHOOT_FAR;
                }
                break;

            case PREPARING_SHOOT_AUTO:
                if (utils.IsInRange(flywheelActualSpeedRPM, flywheelSpeedTarget, FLYWHEEL_TOLERANCE)) {
                    systemState = SystemState.READY_TO_SHOOT_AUTO;
                }
                break;

            case READY_TO_SHOOT_AUTO:
                ballHasPassed = !utils.IsInRange(flywheelActualSpeedRPM, flywheelSpeedTarget, FLYWHEEL_TOLERANCE);
            case READY_TO_SHOOT_NEAR:
            case READY_TO_SHOOT_MID:
            case READY_TO_SHOOT_FAR:
                break;
            default:
                systemState = SystemState.IDLE;
                robot.getTelemetry().addLine("Please enter a valid shooter systemState");
        }
    }

    private void applyFlywheelSpeedWithPIDF(){
        double actualError = flywheelSpeedTarget - flywheelActualSpeedRPM;

        double feedForward = (FLYWHEEL_KF * flywheelSpeedTarget);
        double proportional = actualError * FLYWHEEL_KP;

        double actualTime = PDFTimer.milliseconds();

        if (firstIteration) {
            previousError = actualError;
            firstIteration = false;
        }

        double derivative = FLYWHEEL_KD * (actualError - previousError / actualTime - previousTime);

        previousError = actualError;
        previousTime = actualTime;

        double flywheelPower = proportional + derivative + feedForward;

        flywheelMotor.setPower(
                utils.clamp(
                        utils.getVoltageCompensated(
                                flywheelPower, robot.getVoltage(), 11
                        )
                        , 1, -1
                )//mise en page particuliere la quand meme
        );
    }

    private void updateTargetsDistance(){
        double distance = robot.getCamera().getDistanceMeters();
        flywheelSpeedTarget = 0.0; // the value we will give in fonction of the distance
        hoodPosTarget = 0.0;// the value we will give in fonction of the distance
        //TODO find a way to calculate flywheel and hood targets in fonction of the distance to the goal
    }

    private void telemetry(){
        robot.getTelemetry().addData("Shooter RPM speed", flywheelActualSpeedRPM);
        robot.getTelemetry().addData("Shooter RPM Target speed", flywheelSpeedTarget);
        robot.getTelemetry().addData("Shooter ControlMode", shooterControlMode);
        robot.getTelemetry().addData("Shooter Wanted state", wantedState);
        robot.getTelemetry().addData("Shooter System state", systemState);
    }
}
