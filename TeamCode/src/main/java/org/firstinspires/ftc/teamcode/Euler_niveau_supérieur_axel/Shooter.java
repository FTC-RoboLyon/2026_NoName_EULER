package org.firstinspires.ftc.teamcode.Euler_niveau_supérieur_axel;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class Shooter {
    private DcMotorEx ShooterMotor;
    public double CPR = 28; //fais gaffe ici tu declare variable comme un touble et tu lui donnes une valeur d'int
                            // il aurait soit fallu mettre la variable en int (le mieux ici car le CPR sera toujours un entier) ou alors mettre la valeur a 28.0

    public static double shooterTolerance = 100; //parreil qu'au dessus mais la c'est mieux de mettre 100.0 et non pas changer le type, précise aussi l'unite en commentaire
    public static double shooterKp, shooterKv, shooterKs;
    public static double ShooterPower; //pourquoi il est static lui ?

    public static double posviseur_bank = 0.3, posviseur_mid = 0.58, posviseur_far = 0.45; //TUNEME
    public static double veloNear = 1250, veloMid = 1500, veloFar = 1500; //TUNEME, speed c'est mieux que velo qu'on comprends pas vrm



    public enum ShooterState{
        /*
           Pourquoi tu mets une machine a état alors que tu pourrais directement donner la target en RPM depuis le programme principal
           Si possible refais pour l'instant sans machine a etat et on vera apres pour en ajouter parce que la ca complique ton code
           pour pas grand chose
         */
        VeloShootNear,
        VeloShootMid,
        VeloShootFar,
        Idle;

    }
    private ShooterState shooterState = ShooterState.Idle;
    public void setShooterState(ShooterState shooterState) {
        this.shooterState = shooterState;
    }

    public ShooterState getShooterState() {
        return shooterState;
    }


    public Shooter (HardwareMap hmap){
        ShooterMotor = hmap.get(DcMotorEx.class, "shooter");

        ShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public double get_Shooter_RPM (){
        return (ShooterMotor.getVelocity() / CPR) * 60;     // if you want to use a gear ratio, divide this by the value of the gear ratio
                                                            // -> dans ce cas la mais une constante qu'on peut modifier qui divise tout le temps et pour l'instant laisse la à 1.0
                                                            // Et j'ai l'impression que la conversion de ticks/sec a ticks/min est mal faite
    }
    public double getVoltageCompensated (double power, double voltage){
        return (power*voltage)/13;
    }
    public void SetVeloShooter (double velocity, double voltage){ //mets speed a la place de velo stp et t'as le droit d'aerer un peu ton code pour le rendre plus lisble
        double error = velocity-get_Shooter_RPM(); //c faux l'erreur correspond a la diffecence entre les RPM targets et tes RPM

        if (Math.abs(error) < shooterTolerance)
            error = 0; //plutot que de faire ca mets tout ce qui est apres dans la condition inverse pour eviter de faire des calculs inutiles

        shooterKs = getVoltageCompensated(shooterKs, voltage); //la ton truc ne va pas marcher puisque tu modifie la valeur de base ton kS a chaque fois
        shooterKv = getVoltageCompensated(shooterKv, voltage); //pareil
        double ff = (shooterKv*velocity) + shooterKs;
        double fb = error * shooterKp; //je comprends pas le nom de fb

        ShooterPower = ff + fb; //tu pourrais faire tes calculs sans creer ces deux variables
    }

    public void shooterLoop(double voltage){
        switch (getShooterState()){ //pourquoi tu utilises une fonction pour get une variable a laquelle tu as acces
            case Idle:
                SetVeloShooter(0, voltage); //pourquoi tu passes par ta fonction et ne mais pas direct shooterPower a 0.0
                break;
            case VeloShootNear:
                SetVeloShooter(veloNear, voltage);
                break;
            case VeloShootMid:
                SetVeloShooter(veloMid, voltage);
                break;
            case VeloShootFar:
                SetVeloShooter(veloFar, voltage);
                break;

            //quand tu fais un switch mieux vaut mettre toujours un default pour eviter qu'un beug puisse pprovoquer un comportement innatendu
        }
        ShooterMotor.setPower(getVoltageCompensated(ShooterPower, voltage));
    }
}
