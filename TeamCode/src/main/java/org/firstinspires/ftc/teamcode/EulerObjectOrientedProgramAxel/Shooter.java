package org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Shooter {
    private final DcMotorEx ShooterMotor;
    private final Servo HoodServo;
    private final CRServo transfertServo;
    private int CPR = 28; //fais gafeedForwarde ici tu declare variable comme un double et tu lui donnes une valeur d'int
                            // il aurait soit fallu mettre la variable en int (le mieux ici car le CPR sera toujours un entier) ou alors mettre la valeur a 28.0

    public static double shooterTolerance = 100.0;      //jvais ptet revoir l'unité jsp encore mais pr l'instant c'est en rotaion/minute
    //parreil qu'au dessus mais la c'est mieux de mettre 100.0 et non pas changer le type, précise aussi l'unite en commentaire


    public static double shooterKp, shooterKv, shooterKs;


    public static double posHood_bank = 0.3, posHood_mid = 0.58, posHood_far = 0.45; //TUNEME
    public static double speedNear = 1250, speedMid = 1500, speedFar = 1500; //TUNEME, speed c'est mieux que velo qu'on comprends pas vrm

    private double ShooterPower; //pourquoi il est static lui ?          plus mtn



    //public enum ShooterState{
    //    /*
    //       Pourquoi tu mets une machine a état alors que tu pourrais directement donner la target en RPM depuis le programme principal
    //       Si possible refais pour l'instant sans machine a etat et on vera apres pour en ajouter parce que la ca complique ton code
    //       pour pas grand chose
    //     */
    //    VeloShootNear,
    //    VeloShootMid,
    //    VeloShootFar,
    //    Idle;
//
    //}
    //private ShooterState shooterState = ShooterState.Idle;
    //public void setShooterState(ShooterState shooterState) {
    //    this.shooterState = shooterState;
    //}
//
    //public ShooterState getShooterState() {
    //    return shooterState;
    //}


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
    }



    public void SetShooterTargets(double speedTarget, double posTarget, double voltage)
    {
        SetFlywheelTargetSpeed(speedTarget, voltage);
        HoodServo.setPosition(posTarget);
    }
    public void SetFlywheelTargetSpeed (double targetSpeed, double voltage){

        //mets speed a la place de velo stp et t'as le droit d'aerer un peu ton code pour le rendre plus lisble        fait
        double error = targetSpeed - get_Shooter_RPM(); //c faux l'erreur correspond a la difeedForwardecence entre les RPM targets et tes RPM

        if (Math.abs(error) < shooterTolerance)
            error = 0; //plutot que de faire ca mets tout ce qui est apres dans la condition inverse pour eviter de faire des calculs inutiles


        //shooterKs = getVoltageCompensated(shooterKs, voltage); //la ton truc ne va pas marcher puisque tu modifie la valeur de base ton kS a chaque fois
        //shooterKv = getVoltageCompensated(shooterKv, voltage); //pareil

        double feedForward = (shooterKv * targetSpeed) + shooterKs;
        double feedBack = error * shooterKp; //je comprends pas le nom de feedBack

        ShooterPower = feedForward + feedBack; //tu pourrais faire tes calculs sans creer ces deux variables
        ShooterMotor.setPower(getVoltageCompensated(ShooterPower, voltage));

        transfertServo.setPower(1);

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
        double gearRatio = 1;    // if you want to use a gear ratio, put it here
        return ((ShooterMotor.getVelocity() / CPR) * 60)/gearRatio;
        // -> dans ce cas la mais une constante qu'on peut modifier qui divise tout le temps et pour l'instant laisse la à 1.0         done
        // Et j'ai l'impression que la conversion de ticks/sec a ticks/min est mal faite
    }

    public void periodic(double voltage){
        //switch (shooterState){ //pourquoi tu utilises une fonction pour get une variable a laquelle tu as acces
        //    case Idle:
        //        ShooterMotor.setPower(0.0); //pourquoi tu passes par ta fonction et ne mais pas direct shooterPower a 0.0
        //        HoodServo.setPosition(posHood_bank);
        //        break;
        //    case VeloShootNear:
        //        SetFlywheelTargetSpeed(speedNear, voltage);
        //        HoodServo.setPosition(posHood_bank);
        //        break;
        //    case VeloShootMid:
        //        SetFlywheelTargetSpeed(speedMid, voltage);
        //        HoodServo.setPosition(posHood_mid);
        //        break;
        //    case VeloShootFar:
        //        SetFlywheelTargetSpeed(speedFar, voltage);
        //        HoodServo.setPosition(posHood_far);
        //        break;
        //    default:
        //        ShooterMotor.setPower(0);
        //        HoodServo.setPosition(posHood_bank);
        //        break;
        //    //quand tu fais un switch mieux vaut mettre toujours un default pour eviter qu'un beug puisse pprovoquer un comportement innatendu
        //}
    }
}
