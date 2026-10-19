package org.firstinspires.ftc.teamcode.CommandBase.Subsystem.Drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.CommandBase.robotContainer;
import org.firstinspires.ftc.teamcode.Lib.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.teamcode.Lib.utils;

import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.Drivetrain.Drivetrain_Constant;

public class DriveTrainSubsystem extends SubsystemBase {

    private final Drivetrain_hardware drivetrainHardware;

    private ElapsedTime goToPosTimer = new ElapsedTime();
    private ElapsedTime headingTimer = new ElapsedTime();

    private DoubleSupplier ySupplier, xSupplier, turnSupplier;
    private robotContainer robot;

    private double previousGoPosTime = 0.0;
    private double previousHeadingTime = 0.0;

    private double previousFwdError = 0;
    private double previousStrafeError = 0;
    private double previousHeadingError = 0;
    private boolean PDfirstIteration = true; //stay true until first iteration is finished
    // for the boolean above, stay true until first iteration of goToPos() or driveHeading() (become false a this moment)
    // and become true again when target of the function is reached


    private double xTarget, yTarget, headingTarget;

    private double xPower = 0.0, yPower = 0.0, rotationPower = 0.0;



    public enum DriveMode{
        DISABLE,
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
        GO_TO_POS,
        DRIVE_AND_HEAD_TO_TARGET
    }
    private DriveMode driveMode = DriveMode.DISABLE;
    public void setDriveMode(DriveMode drive){
        driveMode = drive;
    }
    public DriveMode getDriveMode(){return driveMode;}


    public DriveTrainSubsystem (HardwareMap hmap,
                                robotContainer robot
                                ){

        if (ySupplier == null)
            ySupplier = ()->0.0;

        if (xSupplier == null)
            xSupplier = ()->0.0;

        if (turnSupplier == null)
            turnSupplier = ()->0.0;

        this.robot = robot;
        drivetrainHardware = new Drivetrain_hardware(hmap);

        xTarget = Drivetrain_Constant.blueStartingPos1.xMeters;
        yTarget = Drivetrain_Constant.blueStartingPos1.yMeters;
        headingTarget = Drivetrain_Constant.blueStartingPos1.headingRadians;
        //TODO a way to select the starting Pos


        goToPosTimer.startTime();
        goToPosTimer.reset();

        headingTimer.startTime();
        headingTimer.reset();

    }

    public DriveTrainSubsystem (HardwareMap hmap,
                                robotContainer robot,
                                Pose2d startPos){
        this(hmap, robot);
        drivetrainHardware.setPose(startPos);

    }


    public void setSupplier(DoubleSupplier ySupplier,
                            DoubleSupplier xSupplier,
                            DoubleSupplier turnSupplier){
        this.ySupplier = ySupplier;
        this.xSupplier = xSupplier;
        this.turnSupplier = turnSupplier;
    }



    /**
     * A function that allows the robot to move to a given point of coordinates (xTarget, yTarget) and head to a given heading target.
     * Return if the robot has arrived yet using tolerances.
     * @param Xtarget X coordinate of the target point (in meters)
     * @param Ytarget Y coordinate of the target point (in meters)
     * @param Headingtarget heading target of the robot (in radians)
     * @return if the robot has arrived yet using tolerances (true : yes; false : no)
     */
    public void setGoToPosTargets (double Xtarget, double Ytarget, double Headingtarget) {
        xTarget = Xtarget;
        yTarget = Ytarget;
        headingTarget = Headingtarget;

        PDfirstIteration = true;
    }

    public boolean hasReachedLinearTargets(){
        return utils.IsInRange(drivetrainHardware.getRobotX(), xTarget, Drivetrain_Constant.TOLERANCE_X_AND_Y)
                && utils.IsInRange(drivetrainHardware.getRobotY(), yTarget, Drivetrain_Constant.TOLERANCE_X_AND_Y);
    }

    public boolean alineWithHeadingTarget(){
        if (driveMode == DriveMode.DRIVE_AND_HEAD_TO_TARGET || driveMode == DriveMode.GO_TO_POS)
            return utils.IsInRange(drivetrainHardware.getRobotHeading(), headingTarget, Drivetrain_Constant.TOLERANCE_HEADING);
        else
            return true;
    }


    @Override
    public void periodic(){

        drivetrainHardware.update();
        double headingError;

        switch (driveMode){
            case DISABLE:
                xPower = 0.0;
                yPower = 0.0;
                rotationPower = 0.0;
                if (!PDfirstIteration)
                    PDfirstIteration = true;
                break;

            case ROBOT_CENTRIC:
                xPower = xSupplier.getAsDouble();
                yPower = ySupplier.getAsDouble();
                rotationPower = turnSupplier.getAsDouble();

                break;

            case FIELD_CENTRIC:

                xPower = xSupplier.getAsDouble();
                yPower = ySupplier.getAsDouble();
                rotationPower = turnSupplier.getAsDouble();

                double xPower1 = xPower;
                xPower = Math.cos(drivetrainHardware.getRobotHeading()) * xPower1 + Math.sin(drivetrainHardware.getRobotHeading())* yPower;
                yPower = -Math.sin(drivetrainHardware.getRobotHeading()) * xPower1 + Math.cos(drivetrainHardware.getRobotHeading())* yPower;

                break;

            case GO_TO_POS:

                headingError = headingTarget - drivetrainHardware.getRobotHeading();

                double xError = xTarget - drivetrainHardware.getRobotX();
                double yError = yTarget - drivetrainHardware.getRobotY();

                double fwdError= Math.cos(drivetrainHardware.getRobotHeading()) * xError + Math.sin(drivetrainHardware.getRobotHeading()) * yError;
                double strafeError = -Math.sin(drivetrainHardware.getRobotHeading()) * xError + Math.cos(drivetrainHardware.getRobotHeading()) * yError;

                double pTermX = Drivetrain_Constant.KP_FORWARD * fwdError;
                double pTermY = Drivetrain_Constant.KP_STRAFE * strafeError;
                double pTermHeading1 = Drivetrain_Constant.KD_HEADING * headingError;

                double currentTime = goToPosTimer.milliseconds();

                if (PDfirstIteration){
                    previousFwdError = fwdError;
                    previousStrafeError = strafeError;
                    previousHeadingError = headingError;
                    PDfirstIteration = false;
                }
                double dTermX = Drivetrain_Constant.KD_FORWARD * ((fwdError - previousFwdError)/(currentTime - previousGoPosTime));
                double dTermY = Drivetrain_Constant.KD_STRAFE * ((strafeError - previousStrafeError)/(currentTime - previousGoPosTime));
                double dTermHeading1 = Drivetrain_Constant.KD_HEADING * ((headingError - previousHeadingError)/(currentTime - previousGoPosTime));

                xPower = pTermX + dTermX;
                yPower = pTermY + dTermY;
                rotationPower = pTermHeading1 + dTermHeading1;

                previousFwdError = fwdError;
                previousStrafeError = strafeError;
                previousHeadingError = headingError;
                previousGoPosTime = currentTime;

                break;

            case DRIVE_AND_HEAD_TO_TARGET:

                headingTarget = drivetrainHardware.getRobotHeading() - robot.getCamera().getBearing();

                headingError = headingTarget - drivetrainHardware.getRobotHeading();

                double pTermHeading2 = Drivetrain_Constant.KP_HEADING * headingError;

                if (PDfirstIteration){
                    previousHeadingError = headingError;
                    PDfirstIteration = false;
                }

                double actualTime = headingTimer.milliseconds();
                double dTermHeading2 = Drivetrain_Constant.KD_HEADING * ((headingError - previousHeadingError)/(actualTime - previousHeadingTime));

                xPower = xSupplier.getAsDouble();
                yPower = ySupplier.getAsDouble();
                rotationPower = pTermHeading2 + dTermHeading2;

                double xPower2 = xPower;
                xPower = Math.cos(drivetrainHardware.getRobotHeading())* xPower2 + Math.sin(drivetrainHardware.getRobotHeading())* yPower;
                yPower = -Math.sin(drivetrainHardware.getRobotHeading())* xPower2 + Math.cos(drivetrainHardware.getRobotHeading())* yPower;

                previousHeadingError = headingError;
                previousHeadingTime = actualTime;

                break;

            default:
                driveMode = DriveMode.DISABLE;
                robot.getTelemetry().addLine("Please enter a valid drivetrain driveMode");
                break;

        }

        drivetrainHardware.applyMotorsPower(xPower, yPower, rotationPower);
    }
}
