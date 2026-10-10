package org.firstinspires.ftc.teamcode.CommandBase.Subsystem;
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
    private BooleanSupplier plusVeloSupplier;
    private BooleanSupplier minusVeloSupplier;
    private BooleanSupplier plusIncrementationSupplier;
    private BooleanSupplier minusIncrementationSupplier;
    private int veloIncrementation;

    public static final double GEAR_RATIO = 1.0;
    public static final double TICKS_PER_ROTATION = 28;

    public static final double FLYWHEEL_KP = 1.0, FLYWHEEL_KF = 1.0, FLYWHEEL_KD = 1.0; //TUNEME

    public static final double HOOD_TOLERANCE = 100.0;  //TUNEME between 0 and 1 ? 0 and 1 what ? potatoes, chairs, Antoines, ;)   mdrr no but it is so long to write "between 1 and 0 the percentage of the plage your servo can parcourate"
    public static final double FLYWHEEL_TOLERANCE = 100.0;  //TUNEME in RPM

    public static final double NEAR_POS_HOOD = 0.3, MID_POS_HOOD = 0.58, FAR_POS_HOOD = 0.45; //TUNEME between 0 and 1
    public static final double NEAR_FLYWHEEL_RPM = 1250, MID_FLYWHEEL_RPM = 1500, FAR_FLYWHEEL_RPM = 1500; //TUNEME in RPM

    private static double flywheelRPM = 0.0;
    private static double flywheelVeloTarget = 0.0;
    private static double hoodPosTarget = 0.0;
    private static double previousError = 0.0;
    private static double previousTime = 0.0;
    private boolean firstIteration = true;

    public enum WantedState {
        STAND_BY,
        SHOOT_NEAR,
        SHOOT_MID,
        SHOOT_FAR,
        //En fait si tu regarde bien mon code de SecretProject les machines a etat (en tout cas de Wanted State et System State)
        // ne sont utilisée que lorsque le systeme est en mode automatique. Pour savoir ça j'utilise le ControlMode que tu peux trouver dans mon code
        // jsp pas pk Adam l'a pas encore remise dans la derniere version de la LyonLib mais de toute maniere il m'a dit qu'il devrait bientot push ses dernieres modifs
        // mais pour l'instant t'a qu'à utiliser la version qu'est dans mon Secret Project (plus si secret d'ailleurs)
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

    public void setTargets(double flywheelTarget, double HoodPos) {
        flywheelVeloTarget = flywheelTarget;
        hoodPosTarget = HoodPos;
    }//PLus vrm censé en avoir besoin puisque tes commandes sont juste censées changer Wanted State et/ou evetuellement Control Mode
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


        if (plusVeloSupplier == null) {
            plusVeloSupplier = new BooleanSupplier() {
                @Override
                public boolean getAsBoolean() {
                    robot.getTelemetry().addLine("Please define your supplier");
                    return false;
                }};}
        if (minusVeloSupplier == null) {
            minusVeloSupplier = new BooleanSupplier() {
                @Override
                public boolean getAsBoolean() {
                    robot.getTelemetry().addLine("Please define your supplier");
                    return false;
                }};}
        if (plusIncrementationSupplier == null) {
            plusIncrementationSupplier = new BooleanSupplier() {
                @Override
                public boolean getAsBoolean() {
                    robot.getTelemetry().addLine("Please define your supplier");
                    return false;
                }
            };}
        if (minusIncrementationSupplier == null) {
            minusIncrementationSupplier = new BooleanSupplier() {
                @Override
                public boolean getAsBoolean() {
                    robot.getTelemetry().addLine("Please define your supplier");
                    return false;
                }};}
    }

    public void setSupplier(BooleanSupplier plusVeloSupplier,
                            BooleanSupplier minusVeloSupplier,

                            BooleanSupplier plusIncrementationSupplier,
                            BooleanSupplier minusIncrementationSupplier){

        this.plusVeloSupplier = plusVeloSupplier;
        this.minusVeloSupplier = minusVeloSupplier;

        this.plusIncrementationSupplier = plusIncrementationSupplier;
        this.minusIncrementationSupplier = minusIncrementationSupplier;
    }


    public void stopShooter(){
        flywheelMotor.setPower(0.0);
        hoodServo.setPosition(MID_POS_HOOD);
    }


    @Override
    public void periodic(){

        switch (shooterControlMode) {
            case DISABLED:
                break;
            case MANUAL_VELOCITY:
                if (plusVeloSupplier.getAsBoolean())
                    flywheelVeloTarget += veloIncrementation;
                if (minusVeloSupplier.getAsBoolean())
                    flywheelVeloTarget -= veloIncrementation;

                if (plusIncrementationSupplier.getAsBoolean())
                    veloIncrementation *= 10;
                if(minusIncrementationSupplier.getAsBoolean())
                    veloIncrementation /= 10;

                applyFlywheelVeloWithPIDF();
                hoodServo.setPosition(hoodPosTarget);
                break;

            case VELOCITY_VOLTAGE_PIDF:

                flywheelRPM = utils.TickPerSecondToRPM(flywheelMotor.getVelocity(), TICKS_PER_ROTATION, GEAR_RATIO);

                RunStateMachine();

                switch (systemState) {
                    case IDLE:
                        stopShooter();
                        break;

                    case PREPARING_SHOOT_AUTO:
                    case READY_TO_SHOOT_AUTO:
                        updateTargetsDistance();
                        applyFlywheelVeloWithPIDF();
                        hoodServo.setPosition(hoodPosTarget);
                        break;

                    case PREPARING_SHOOT_NEAR:
                    case PREPARING_SHOOT_MID:
                    case PREPARING_SHOOT_FAR:
                    case READY_TO_SHOOT_NEAR:
                    case READY_TO_SHOOT_MID:
                    case READY_TO_SHOOT_FAR:
                        applyFlywheelVeloWithPIDF();
                        hoodServo.setPosition(hoodPosTarget);
                        break;

                    default:
                        systemState = SystemState.IDLE;
                        break;
                }
                break;
        }
    }

    private void RunStateMachine(){
        switch (wantedState)
        {
            case STAND_BY:
                systemState = SystemState.IDLE;
                break;

            case SHOOT_NEAR:
                flywheelVeloTarget = NEAR_FLYWHEEL_RPM;
                hoodPosTarget = NEAR_POS_HOOD;
                if (systemState != SystemState.READY_TO_SHOOT_NEAR)
                {
                    systemState = SystemState.PREPARING_SHOOT_NEAR;
                }
                firstIteration = true;
                break;

            case SHOOT_MID:
                flywheelVeloTarget = MID_FLYWHEEL_RPM;
                hoodPosTarget = MID_POS_HOOD;
                if (systemState != SystemState.READY_TO_SHOOT_MID)
                {
                    systemState = SystemState.PREPARING_SHOOT_MID;
                }
                firstIteration = true;
                break;

            case SHOOT_FAR:
                flywheelVeloTarget = FAR_FLYWHEEL_RPM;
                hoodPosTarget = FAR_POS_HOOD;
                if (systemState != SystemState.READY_TO_SHOOT_FAR)
                {
                    systemState = SystemState.PREPARING_SHOOT_FAR;
                }
                break;

            case SHOOT_AUTO:
                updateTargetsDistance();
                if (systemState != SystemState.READY_TO_SHOOT_AUTO)
                {
                    systemState = SystemState.PREPARING_SHOOT_AUTO;
                }
                firstIteration = true;
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
                if (utils.IsInRange(flywheelRPM, flywheelVeloTarget, FLYWHEEL_TOLERANCE) && utils.IsInRange(hoodServo.getPosition(), hoodPosTarget, HOOD_TOLERANCE))
                {
                    systemState = SystemState.READY_TO_SHOOT_NEAR;
                }
                break;

            case PREPARING_SHOOT_MID:
                if (utils.IsInRange(flywheelRPM, flywheelVeloTarget, FLYWHEEL_TOLERANCE) && utils.IsInRange(hoodServo.getPosition(), hoodPosTarget, HOOD_TOLERANCE))
                {
                    systemState = SystemState.READY_TO_SHOOT_MID;
                }
                break;

            case PREPARING_SHOOT_FAR:
                if (utils.IsInRange(flywheelRPM, flywheelVeloTarget, FLYWHEEL_TOLERANCE) && utils.IsInRange(hoodServo.getPosition(), hoodPosTarget, HOOD_TOLERANCE))
                {
                    systemState = SystemState.READY_TO_SHOOT_FAR;
                }
                break;

            case PREPARING_SHOOT_AUTO:
                if (utils.IsInRange(flywheelRPM, flywheelVeloTarget, FLYWHEEL_TOLERANCE) && utils.IsInRange(hoodServo.getPosition(), hoodPosTarget, HOOD_TOLERANCE))
                {
                    systemState = SystemState.READY_TO_SHOOT_AUTO;
                }
                break;

            case READY_TO_SHOOT_AUTO:
            case READY_TO_SHOOT_NEAR:
            case READY_TO_SHOOT_MID:
            case READY_TO_SHOOT_FAR:
                break;
            default:
                systemState = SystemState.IDLE;
                robot.getTelemetry().addLine("Please enter a valid shooter systemState");
        }
    }

    private void applyFlywheelVeloWithPIDF() {
        double actualError = flywheelVeloTarget - flywheelRPM;

        double feedForward = (FLYWHEEL_KF * flywheelVeloTarget);
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
                )
        );
    }

    private void updateTargetsDistance(){
        double distance = robot.getCamera().getDistanceMeters();
        flywheelVeloTarget = 0.0; // the value we will give in fonction of the distance
        hoodPosTarget = 0.0;// the value we will give in fonction of the distance
        //TODO find a way to calculate flywheel and hood targets in fonction of the distance to the goal
    }
}
