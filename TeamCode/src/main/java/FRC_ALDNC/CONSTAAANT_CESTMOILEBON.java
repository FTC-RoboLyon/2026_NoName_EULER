package FRC_ALDNC;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CONSTAAANT_CESTMOILEBON {
    public static String LEFT_MOTOR = "left motor", RIGHT_MOTOR = "right motor", INTAKE = "intake", SHOOTER = "shooter", FEEDER = "feeder", VISEUR = "viseur", COMPTEUR_BALLE = "compteur balles";
    public static double ShooterKP= 1400, ShooterKI = 0.0, ShooterKD = 0.0; //TUNEME
    public static double rotation_P,rotation_I,rotation_D = 0 ; //tunme
    public static double rotation_tolerance = 0 ; //tunme
    public static double posFeed = 0.3, posFeedrepos = 0.125 ; //tunme
    public static double posviseur_bank = 1, posviseur_mid = 0.6, posviseur_far = 0.5; //TUNEME
    public static double velo_shoot_bank = 900, velo_shoot_mid = 1200, velo_shoot_far = 1400; //TUNEME
    public  static  double shooter_velo_tolerance = 10; //Tunme
    public static double seuil_volt_shooter = 11; //TUNEME
    public static double coef_smooth_stickleft = 0.25, deadzone_sticktleft = 0.05, vpower_stickleft = 2; //TUNEME
    public static double coef_smooth_stickright = 0.25, deadzone_stickright = 0.05, vpower_stickright = 2; //TUNEME
    public  static  double distance_ou_le_capteur_detecte_balle = 30; //TUNEME
    public  static  double time_to_let_a_ball_pass = 500; //TUNEME IN MILLISECOND

}
