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

    public static double shooterKp, shooterKv, shooterKs; //et c'est ou que tu leur donne une valeur a eux


    public static double posHood_bank = 0.3, posHood_mid = 0.58, posHood_far = 0.45; //TUNEME -> inverser pos et ce qui suit dans les noms c'est plus correct et respecter les convention de nommage (ex hoodBankPos)
    public static double speedNear = 1250, speedMid = 1500, speedFar = 1500; //TUNEME -> inverser speed et ce qui suit dans les noms c'est plus correct (ex nearSpeed)

    private InterpLUT FlywheelLUT; //pk une interpolation sur une spline et pas plutot lineaire ? et ajouter a quelle grandeur ca correspond (FlywheelSpeedLUT)
    private InterpLUT HoodLUT; //pk une interpolation sur une spline et pas plutot lineaire ? et ajouter a quelle grandeur ca correspond (HoodPosLUT)

    private double ShooterPower;


    public Shooter (HardwareMap hmap){
        ShooterMotor = hmap.get(DcMotorEx.class, "shooter");

        ShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        ShooterMotor.setPower(0.0);

        HoodServo = hmap.get(Servo.class, "viseur");
        HoodServo.setPosition(posHood_bank);

        transfertServo = hmap.get(CRServo.class, "chemin");
        transfertServo.setPower(0);

        FlywheelLUT = new InterpLUT();
        FlywheelLUT.add(0,0); //add as much as you want  FlywheelLUT.add(input, output)

        HoodLUT = new InterpLUT();
        HoodLUT.add(0,0); //add as much as you want  HoodLUT.add(input, output)
    }



    public void SetShooterTargets(double speedTarget, double posTarget, double voltage)
    {
        SetFlywheelTargetSpeed(speedTarget, voltage);
        HoodServo.setPosition(posTarget);
    }

    public void SetShooterTargetAutomaticly (double distanceToGoal, double voltage){ //On dit Automatically et pas Automaticly et le nom n'est pas fou, tu pourrais juste mettre SetShooterDistanceTarget() c mieux
        double speedTarget = FlywheelLUT.get(distanceToGoal);
        double posTarget = HoodLUT.get(distanceToGoal);

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

        transfertServo.setPower(1); //et lui il repasse jamais a 0 dcp
    }

    public void stopShooter(){
        ShooterMotor.setPower(0.0);
        HoodServo.setPosition(posHood_bank);
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
