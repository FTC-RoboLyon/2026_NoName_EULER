package FRC_ALDNC;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CONSTAAANT_CESTMOILEBON {
    public static String LEFT_MOTOR = "left motor", RIGHT_MOTOR = "right motor", INTAKE = "intake", SHOOTER = "shooter", FEEDER = "feeder", VISEUR = "viseur", COMPTEUR_BALLE = "compteur balles", ENCODERD = "encoderD", ENCODEURG = "encoderG";

    public static double ShooterKP= 0.005, ShooterKI = 0.0, ShooterKD = 0.005, ShooterKF = 0.003825; //TUNEME
    public static double ShooterKP_velo= 100, ShooterKI_velo = 0.5, ShooterKD_velo = 0, ShooterKF_velo = 16; //TUNEME

    public  static  double shooter_velo_tolerance = 100; //Tunme
    public  static  double shooter_timestamp = 20; //Tunme

    public static double posviseur_bank = 1, posviseur_mid = 0.58, posviseur_far = 0.45; //TUNEME
    public static double velo_shoot_bank = 975, velo_shoot_mid = 1250, velo_shoot_far = 1500; //TUNEME

    public static double seuil_volt_shooter = 11; //TUNEME

    public static double shooter_aspirage_puissance = -400;

    public static double ff_distance = 0.025; //TUNEME
    public static double tolerance_distance = 6; //TUNEME
    public static double kp_distance = 0.005; //TUNEME


    public static double ff_rotation = 6.43;
    public static double p_rotation = 0.0025;
    public static double tolerence_rotation = 0.25;
    public  static double seuilDriveShooter = 0.05;
    public  static double tolerance_go_angle = 1;

    public static double rotation_P,rotation_I,rotation_D = 0 ; //tunme
    public static double rotation_tolerance = 0 ; //tunme

    public  static double forward_auto = -1;
    public  static double turn_auto = 0;
    public  static double time_auto = 1000;


    public static double posFeed = 0.33, posFeedrepos = 0.125 ; //tunme
    public  static  double time_to_let_a_ball_pass = 2250; //TUNEME IN MILLISECOND



    public static double coef_smooth_stickleft = 0.45, deadzone_sticktleft = 0, vpower_stickleft = 1; //TUNEME
    public static double coef_smooth_stickright = 0.45, deadzone_stickright =0, vpower_stickright = 1; //TUNEME


    public  static  double distance_ou_le_capteur_detecte_balle = 30; //TUNEME


}
