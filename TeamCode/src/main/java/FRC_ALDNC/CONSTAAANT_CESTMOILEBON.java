package FRC_ALDNC;

//import com.acmerobotics.dashboard.config.Config;

//@Config
public class CONSTAAANT_CESTMOILEBON {
    public static String LEFT_MOTOR = "left motor", RIGHT_MOTOR = "right motor", INTAKE = "intake", SHOOTER = "shooter", FEEDER = "feeder", VISEUR = "viseur", COMPTEUR_BALLE = "compteur balles", ENCODERD = "encoderD", ENCODEURG = "encoderG";

    public static double ShooterKP= 0.005, ShooterKI = 0.0, ShooterKD = 0.005, ShooterKF = 0.003825; //TUNEME
    public static double ShooterKP_velo= 0.005, ShooterKI_velo = 0.0, ShooterKD_velo = 0.005, ShooterKF_velo = 16; //TUNEME
    public  static  double shooter_velo_tolerance = 10; //Tunme
    public  static  double shooter_timestamp = 20; //Tunme
    public static double posviseur_bank = 1, posviseur_mid = 0.6, posviseur_far = 0.5; //TUNEME
    public static double velo_shoot_bank = 900, velo_shoot_mid = 1200, velo_shoot_far = 1400; //TUNEME
    public static double seuil_volt_shooter = 11; //TUNEME
    public static double shooter_aspirage_puissance = -200;

    public static double ff_rotation = 6.43;
    public static double p_rotation = 0.002;
    public static double tolerence_rotation = 20;
    public  static double seuilDriveShooter = 0.05;
    public  static double tolerance_go_angle = 1;


    public static double rotation_P,rotation_I,rotation_D = 0 ; //tunme
    public static double rotation_tolerance = 0 ; //tunme


    public static double posFeed = 0.3, posFeedrepos = 0.125 ; //tunme
    public  static  double time_to_let_a_ball_pass = 500; //TUNEME IN MILLISECOND



    public static double coef_smooth_stickleft = 1, deadzone_sticktleft = 0, vpower_stickleft = 2; //TUNEME
    public static double coef_smooth_stickright = 1, deadzone_stickright =0, vpower_stickright = 2; //TUNEME


    public  static  double distance_ou_le_capteur_detecte_balle = 30; //TUNEME


}
