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

public class ShooterSubsystem extends SubsystemBase {
    private DcMotorEx flywheelMotor;
    private Servo hoodServo;
    private ElapsedTime PDFTimer = new ElapsedTime();
    private robotContainer robot;

    public static final double GEAR_RATIO = 1.0;
    public static final double TICKS_PER_ROTATION = 28;

    public static final double FLYWHEEL_KP = 1.0, FLYWHEEL_KF = 1.0, FLYWHEEL_KD = 1.0; //TUNEME

    public static final double HOOD_TOLERANCE = 100.0;  //TUNEME between 0 and 1 ? 0 and 1 what ? potatoes, chairs, Antoines, ;)
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
        MANUAL,
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
        PREPARING_SHOOT_MANUAL,
        PREPARING_SHOOT_AUTO,

        READY_TO_SHOOT_MANUAL,
        READY_TO_SHOOT_AUTO,
        READY_TO_SHOOT_NEAR,
        READY_TO_SHOOT_MID,
        READY_TO_SHOOT_FAR
    }
    private WantedState wantedState = WantedState.STAND_BY;
    private SystemState systemState = SystemState.IDLE;
    public void setWantedState (WantedState state) {
        wantedState = state;
    }

    public void setTargets(double flywheelTarget, double HoodPos) {
        flywheelVeloTarget = flywheelTarget;
        hoodPosTarget = HoodPos;
    }//PLus vrm censé en avoir besoin puisque tes commandes sont juste censées changer Wanted State et/ou evetuellement Control Mode
    public SystemState getSystemState() {return systemState;}


    public ShooterSubsystem(HardwareMap hmap, String shooterName, String hoodName, robotContainer robotContainer){
        flywheelMotor = hmap.get(DcMotorEx.class, shooterName);
        hoodServo = hmap.get(Servo.class, hoodName);
        robot = robotContainer;

        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Force au zeroPowerBehavior que tu as oublie

        PDFTimer.startTime();
        PDFTimer.reset();
    }


    public void stopShooter(){
        flywheelMotor.setPower(0.0);
        hoodServo.setPosition(MID_POS_HOOD);
    }


    @Override
    public void periodic(){

        flywheelRPM = utils.TickPerSecondToRPM(flywheelMotor.getVelocity(), TICKS_PER_ROTATION, GEAR_RATIO);

        RunStateMachine();

        switch (systemState)
        {
            case IDLE:
                stopShooter();
                break;
            case PREPARING_SHOOT_AUTO:
                //TODO find a way to find a flywheel velocity and a hood position in fonction of robot.getCameraDistanceToGoal()
            case PREPARING_SHOOT_NEAR:
            case PREPARING_SHOOT_MID:
            case PREPARING_SHOOT_FAR:
            case PREPARING_SHOOT_MANUAL:

                double flywheelPower = calculateFlywheelPower();
                flywheelMotor.setPower(
                        utils.clamp(
                            utils.getVoltageCompensated(
                                    flywheelPower, robot.getVoltage(), 11
                            )
                        ,1, -1
                    )
                );
                hoodServo.setPosition(hoodPosTarget);

            case READY_TO_SHOOT_MANUAL:
            case READY_TO_SHOOT_NEAR:
            case READY_TO_SHOOT_MID:
            case READY_TO_SHOOT_FAR:
                //ah oui donc toi tu t'es cru en exo de physique a negliger les frottements quoi ;)
                break;
            default:
                systemState = SystemState.IDLE;
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
                systemState = SystemState.PREPARING_SHOOT_NEAR; //-> donc meme si il est deja en READY_TO_SHOOT_NEAR tu le rechange tjr en PREPARING, tu tireras donc jamais
                break;

            case SHOOT_MID:
                flywheelVeloTarget = MID_FLYWHEEL_RPM; //same
                hoodPosTarget = MID_POS_HOOD;
                systemState = SystemState.PREPARING_SHOOT_MID;
                break;

            case SHOOT_FAR:
                flywheelVeloTarget = FAR_FLYWHEEL_RPM; //same
                hoodPosTarget = FAR_POS_HOOD;
                systemState = SystemState.PREPARING_SHOOT_FAR;
                break;

            case MANUAL:
                systemState = SystemState.PREPARING_SHOOT_MANUAL; //same
                break;
            case SHOOT_AUTO:
                systemState = SystemState.PREPARING_SHOOT_AUTO; //same
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
                    firstIteration = true;
                }
                break;

            case PREPARING_SHOOT_MID:
                if (utils.IsInRange(flywheelRPM, flywheelVeloTarget, FLYWHEEL_TOLERANCE) && utils.IsInRange(hoodServo.getPosition(), hoodPosTarget, HOOD_TOLERANCE))
                {
                    systemState = SystemState.READY_TO_SHOOT_MID;
                    firstIteration = true;//pk repasser ca en true, tu coupes le PID quand tu es a la bonne vitesse
                }
                break;

            case PREPARING_SHOOT_FAR:
                if (utils.IsInRange(flywheelRPM, flywheelVeloTarget, FLYWHEEL_TOLERANCE) && utils.IsInRange(hoodServo.getPosition(), hoodPosTarget, HOOD_TOLERANCE))
                {
                    systemState = SystemState.READY_TO_SHOOT_FAR;
                    firstIteration = true; //same
                }
                break;

            case PREPARING_SHOOT_MANUAL:
                if (utils.IsInRange(flywheelRPM, flywheelVeloTarget, FLYWHEEL_TOLERANCE) && utils.IsInRange(hoodServo.getPosition(), hoodPosTarget, HOOD_TOLERANCE))
                {
                    systemState = SystemState.READY_TO_SHOOT_MANUAL;
                    firstIteration = true; //same
                }
                break;
            case PREPARING_SHOOT_AUTO:
                if (utils.IsInRange(flywheelRPM, flywheelVeloTarget, FLYWHEEL_TOLERANCE) && utils.IsInRange(hoodServo.getPosition(), hoodPosTarget, HOOD_TOLERANCE))
                {
                    systemState = SystemState.PREPARING_SHOOT_AUTO;
                    firstIteration = true; //same
                }
                break;


            case READY_TO_SHOOT_MANUAL:
            case READY_TO_SHOOT_NEAR:
            case READY_TO_SHOOT_MID:
            case READY_TO_SHOOT_FAR:
                break;
            default:
                systemState = SystemState.IDLE;
                robot.getTelemetry().addLine("Please enter a valid shooter systemState");
                //boucle infiniiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiieeeeeeeeeeeeeeeeeeeeeeeeeeeee
        }
    }

    private double calculateFlywheelPower(){
        double actualError = flywheelVeloTarget - flywheelRPM;

        double feedForward = (FLYWHEEL_KF * flywheelVeloTarget);
        double proportional = actualError * FLYWHEEL_KP;

        double actualTime = PDFTimer.milliseconds();

        if (firstIteration){
            previousError = actualError;
            firstIteration = false;
        }

        double derivative = FLYWHEEL_KD * (actualError - previousError / actualTime - previousTime);

        previousError = actualError;
        previousTime = actualTime;

        return proportional + derivative + feedForward;
    }
}
