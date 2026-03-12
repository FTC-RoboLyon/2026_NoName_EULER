package FRC_ALDNC.CONSTANT;

import com.acmerobotics.dashboard.config.Config;

@Config

public class constante_joystick_and_base {
    public static String LEFT_MOTOR = "left motor", RIGHT_MOTOR = "right motor";

    public static double rotation_P,rotation_I,rotation_D = 0 ; //tunme
    public static double rotation_tolerance = 0 ; //tunme

    public static double ff_rotation = 6.43;
    public static double p_rotation = 0.0025;
    public static double tolerence_rotation = 0.25;
    public  static double seuilDriveShooter = 0.05;
    public  static double tolerance_go_angle = 1;

    public static double coef_smooth_stickleft = 0.45, deadzone_sticktleft = 0, vpower_stickleft = 1; //TUNEME
    public static double coef_smooth_stickright = 0.45, deadzone_stickright =0, vpower_stickright = 1; //TUNEME
}
