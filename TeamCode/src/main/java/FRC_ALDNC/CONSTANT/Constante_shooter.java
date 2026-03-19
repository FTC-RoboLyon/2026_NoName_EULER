package FRC_ALDNC.CONSTANT;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constante_shooter {
    public static String SHOOTER = "shooter", VISEUR = "viseur";

    public static double ShooterKP= 0.005, ShooterKI = 0.0, ShooterKD = 0.005, ShooterKF = 0.003825; //TUNEME
    public static double ShooterKP_velo= 900, ShooterKI_velo = 0, ShooterKD_velo = 0, ShooterKF_velo = 16; //TUNEME

    public  static  double shooter_velo_tolerance = 100; //Tunme
    public  static  double shooter_timestamp = 20; //Tunme

    public static double posviseur_bank = 1, posviseur_mid = 0.58, posviseur_far = 0.45; //TUNEME
    public static double velo_shoot_bank = 975, velo_shoot_mid = 1250, velo_shoot_far = 1500; //TUNEME

    public static double seuil_volt_shooter = 11; //TUNEME

    public static double shooter_aspirage_puissance = -200;
}
