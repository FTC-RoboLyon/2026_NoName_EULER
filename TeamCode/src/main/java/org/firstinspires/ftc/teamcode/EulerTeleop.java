package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.euler.Constant.LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.RIGHT_MOTOR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.euler.Driver;

import java.util.function.BooleanSupplier;

@TeleOp(name = "EulerTeleop", group = "Euler")
public class EulerTeleop extends OpMode {

    private DcMotor left_drive;
    private DcMotor right_drive;
    private DcMotor Intake;
    private DcMotorEx Shooter;


    private Servo viseur;
    private Servo feeder;
    private CRServo chemin;

    public static double posviseur_bank = 0.3, posviseur_mid = 0.58, posviseur_far = 0.45; //TUNEME
    public static double velo_shoot_bank = 1250, velo_shoot_mid = 1500, velo_shoot_far = 1500; //TUNEME

    private double actual_X, actual_Y;

    public static double posFeed = 0.33, posFeedrepos = 0.16 ; //tunme

    private enum Shooter_wanted_state {
        Shoot_bank,
        Shoot_mid,
        Shoot_far,
        Wait,
        Aspirer
    }
    private Shooter_wanted_state shooter_state = Shooter_wanted_state.Wait;

    private enum Shooter_sys_state {
        WAITING,
        REACHING_TARGET,
        READY
    }
    private Shooter_sys_state shooter_sys_state = Shooter_sys_state.WAITING;
    private double velo_shoot;
    private double pos_viseur;
    private double shooter_tolerance;

    public void Init_motors (){
        left_drive = hardwareMap.get(DcMotorEx.class, "left motor");
        right_drive = hardwareMap.get(DcMotorEx.class, "righr motor");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");

        viseur = hardwareMap.get(Servo.class, "viseur");
        feeder = hardwareMap.get(Servo.class, "feeder");
        chemin = hardwareMap.get(CRServo.class, "chemin");

        left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        right_drive.setDirection(DcMotorSimple.Direction.FORWARD);

        left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void Joystick (){
        double coeff_smooth_forward, vdeadzone_forward, vpower_forward;
        double coeff_smooth_turn, vdeadzone_turn, vpower_turn;
        double New_x = gamepad1.right_stick_x, New_y = gamepad1.left_stick_y;

    }

    public void Drive_loop (double forward, double turn){
        left_drive.setPower(forward - turn);
        right_drive.setPower(forward + turn);
    }
    public void intake_loop (){
        if (gamepad1.right_trigger_pressed)
            Intake.setPower(1);
        if (gamepad1.left_trigger_pressed)
            Intake.setPower(-1);
    }
    public void Shooter_loop (){
        if (gamepad1.bWasPressed())
            shooter_state = Shooter_wanted_state.Shoot_bank;
        else if (gamepad1.yWasPressed())
            shooter_state = Shooter_wanted_state.Shoot_mid;
        else if (gamepad1.aWasPressed())
            shooter_state = Shooter_wanted_state.Shoot_far;
        else if (gamepad1.leftBumperWasPressed())
            shooter_state = Shooter_wanted_state.Aspirer;
        else if (gamepad1.rightBumperWasPressed())
            shooter_state = Shooter_wanted_state.Wait;

        switch (shooter_state){
            case Shoot_bank:
                velo_shoot = velo_shoot_bank;
                pos_viseur = posviseur_bank;
                break;
            case Shoot_mid:
                velo_shoot = velo_shoot_mid;
                pos_viseur = posviseur_mid;
                break;
            case Shoot_far:
                velo_shoot = velo_shoot_far;
                pos_viseur = posviseur_far;
                break;
            case Wait:
                velo_shoot = 0;
                pos_viseur = posviseur_bank;
                break;
            case Aspirer:
                velo_shoot = -200;
                pos_viseur = posviseur_bank;
                break;
        }
        Shooter.setVelocity(velo_shoot);
        viseur.setPosition(pos_viseur);
        //if (velo_shoot - shooter_tolerance < Shooter.getVelocity() && Shooter.getVelocity() < velo_shoot + shooter_tolerance)
          //  shooter_sys_state = Shooter_sys_state.READY;

    }

    @Override
    public void init() {
        Init_motors();
    }

    @Override
    public void loop() {
        Drive_loop(gamepad1.left_stick_y, gamepad1.right_stick_x);
        intake_loop();
        Shooter_loop();
        if (gamepad2.left_bumper)
            chemin.setPower(-1);
        else if (gamepad2.right_bumper)
            chemin.setPower(1);
        

        if (gamepad2.x)
            feeder.setPosition(posFeed);
        else
            feeder.setPosition(posFeedrepos);
    }
}
