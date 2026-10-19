package org.firstinspires.ftc.teamcode.CommandBase.Subsystem.Drivetrain;

import org.firstinspires.ftc.teamcode.Lib.LyonLib.kinematics.Pose2d;

public class Drivetrain_Constant {
    public final static int TICKS_PER_REVOLUTION = 8192 ;
    // Tune this to the number of tick your sensor register per wheel revolution
    public final static double WHEEL_RADIUS = 0.45; //TUNEME in meters
    public final static double METERS_PER_TICK = (WHEEL_RADIUS * Math.PI * 2) / TICKS_PER_REVOLUTION;
    public final static double E = 5.0; // in meters
    public final static double ES = 5.0; //in meters
    public final static double KP_STRAFE = 0.25, KD_STRAFE = 0.25; //TUNEME
    public final static double KP_FORWARD = 0.25, KD_FORWARD = 0.25; //TUNEME
    public final static double KP_HEADING = 0.25, KD_HEADING = 0.25; //TUNEME

    public final static double TOLERANCE_X_AND_Y = 0.05; //TUNEME IN METERS
    public final static double TOLERANCE_HEADING = 0.10; //TUNEME IN RADIANT

    public final static Pose2d blueStartingPos1 = new Pose2d(0.0,0.0,0.0);//TUNEME
    public final static Pose2d blueStartingPos2 = new Pose2d(0.0,0.0,0.0);//TUNEME
    public final static Pose2d redStartingPos1 = new Pose2d(0.0,0.0,0.0); //TUNEME
    public final static Pose2d redStartingPos2 = new Pose2d(0.0,0.0,0.0);//TUNEME
}
