package org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.util.InterpLUT;

@Config
public class Shooter {
    private final DcMotorEx ShooterMotor;
    private final Servo HoodServo;
    private final CRServo transfertServo;
    private int CPR = 28;

    public static double shooterTolerance = 100.0;  //in RPM

    private final double gearRatio = 1.0;

    public static double shooterKp = 1.0, shooterKv = 1.0, shooterKs = 1.0; //TUNEME


    public static double hoodBankPos = 0.3, hoodMidPos = 0.58, hoodFarPos = 0.45; //TUNEME
    public static double speedNear = 1250, speedMid = 1500, speedFar = 1500; //TUNEME

    private InterpLUT FlywheelSpeedLUT; //pk une interpolation sur une spline et pas plutot lineaire ?
    private InterpLUT HoodPosLUT; //pk une interpolation sur une spline et pas plutot lineaire ?

    private double ShooterPower;


    public Shooter (HardwareMap hmap){
        ShooterMotor = hmap.get(DcMotorEx.class, "shooter");

        ShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        ShooterMotor.setPower(0.0);

        HoodServo = hmap.get(Servo.class, "viseur");
        HoodServo.setPosition(hoodBankPos);

        transfertServo = hmap.get(CRServo.class, "chemin");
        transfertServo.setPower(0);

        FlywheelSpeedLUT = new InterpLUT();
        FlywheelSpeedLUT.add(0,0); //add as much as you want  FlywheelSpeedLUT.add(input, output)

        HoodPosLUT = new InterpLUT();
        HoodPosLUT.add(0,0); //add as much as you want  HoodPosLUT.add(input, output)
    }



    public void SetShooterTargets(double speedTarget, double posTarget, double voltage)
    {
        SetFlywheelTargetSpeed(speedTarget, voltage);
        HoodServo.setPosition(posTarget);
    }

    public void SetShooterDistanceTarget (double distanceToGoal, double voltage){
        double speedTarget = FlywheelSpeedLUT.get(distanceToGoal);
        double posTarget = HoodPosLUT.get(distanceToGoal);

        SetShooterTargets(speedTarget, posTarget, voltage);
    }

    public void SetFlywheelTargetSpeed (double targetSpeed, double voltage){

        double error = targetSpeed - get_Shooter_RPM();

        if (Math.abs(error) < shooterTolerance)
            error = 0;

        double feedForward = (shooterKv * targetSpeed) + shooterKs;
        double feedBack = error * shooterKp;

        ShooterPower = feedForward + feedBack;
        ShooterMotor.setPower(getVoltageCompensated(ShooterPower, voltage));

        transfertServo.setPower(1);
    }

    public void stopShooter(){
        ShooterMotor.setPower(0.0);
        HoodServo.setPosition(hoodBankPos);
        transfertServo.setPower(0);
    }

    public double getVoltageCompensated (double power, double voltage){
        double output = (power*voltage)/11;

        if (Math.abs(output) > 1)
            output /= Math.abs(output);

        return output;
    }
    private double get_Shooter_RPM (){
        return ((ShooterMotor.getVelocity() / CPR) * 60)/gearRatio;
    }
}
