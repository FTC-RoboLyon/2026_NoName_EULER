package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.euler.Constant.LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.RIGHT_MOTOR;

import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.euler.Driver;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.function.BooleanSupplier;
@Config
@TeleOp(name = "EulerTeleop", group = "Euler")
public class EulerTeleop extends OpMode {

    private DcMotor left_drive;
    private DcMotor right_drive;
    private DcMotor Intake;
    private DcMotorEx Shooter;


    private Servo viseur;
    private Servo feeder;
    private CRServo chemin;

    public static AprilTagProcessor april_joke;
    private VisionPortal visionPortal;

    public static double posviseur_bank = 0.3, posviseur_mid = 0.58, posviseur_far = 0.45; //TUNEME
    public static double velo_shoot_bank = 1250, velo_shoot_mid = 1500, velo_shoot_far = 1500; //TUNEME

    private double actual_X, actual_Y;
    double nvecteur;
    double angle_joystick;

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
    public static double velo_shoot;
    public static double pos_viseur;
    public static double shooter_tolerance;
    public static double shooter_kp, shooter_kv, shooter_ks;
    public double CPR;

    public double turn, forward;

    public static double tolerance_rotation, kp_rotation, kd_rotation;
    public double error, last_error, actual_time, last_time;

    public  double voltage;
    public VoltageSensor voltageSensor;
    public final ElapsedTime timer_kd_rotation = new ElapsedTime();

    public  void Init_camera (){
        april_joke = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setLensIntrinsics(1666.94, 1666.94, 930.463, 618.081)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(1920, 1080));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setAutoStopLiveView(false);
        builder.addProcessor(april_joke);
        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(april_joke, true);
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);

    }
    public void Init_motors (){
        left_drive = hardwareMap.get(DcMotorEx.class, "left motor");
        right_drive = hardwareMap.get(DcMotorEx.class, "righr motor");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");

        viseur = hardwareMap.get(Servo.class, "viseur");
        feeder = hardwareMap.get(Servo.class, "feeder");
        chemin = hardwareMap.get(CRServo.class, "chemin");

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        right_drive.setDirection(DcMotorSimple.Direction.FORWARD);

        left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        Shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public double get_Shooter_RPM (){
        return (Shooter.getVelocity() / CPR) * 60;     // if yous want to use a gear ratio, divide this by this gear ratio
    }
    public double get_voltage_compensated (double power){
        return (power*voltage)/13;
    }
    public void set_velo_shooter (double velocity){
        double error = velocity-get_Shooter_RPM();
        if (Math.abs(error) < shooter_tolerance)
            error = 0;
        shooter_ks = get_voltage_compensated(shooter_ks);
        shooter_kv = get_voltage_compensated(shooter_kv);
        double ff = (shooter_kv*velocity) + shooter_ks;
        double fb = error * shooter_kp;

        Shooter.setPower(get_voltage_compensated(ff + fb));
        telemetry.addData("erreur", error);
        telemetry.update();
    }
    public void Joystick (){
        double coeff_smooth_forward = 0.45, vpower_forward = 2;
        double coeff_smooth_turn = 0.45, vpower_turn = 2;
        double r = 1;

        double New_x = gamepad1.right_stick_x, New_y = gamepad1.left_stick_y;
        double ancienX = New_x, ancienY = New_y;
        nvecteur = Math.sqrt(New_x*New_x + New_y*New_y);
        angle_joystick = Math.atan(New_y/New_x);

        if (nvecteur > r){
            New_x -= Math.cos(angle_joystick)*nvecteur;
            New_y -= Math.sin(angle_joystick)*nvecteur;
        }
        New_x = ancienX + (New_x-ancienX)*coeff_smooth_turn;
        New_y = ancienY + (New_y-ancienY)*coeff_smooth_forward;

        New_x = New_x >= 0 ? Math.pow(New_x, vpower_turn) : -Math.pow(New_x, vpower_turn);
        New_y = New_y >= 0 ? Math.pow(New_y, vpower_forward) : -Math.pow(New_y, vpower_forward);

        actual_X = New_x;
        actual_Y = New_y;

    }

    public void Drive_loop (double forward, double turn){
        left_drive.setPower(get_voltage_compensated(forward - turn));
        right_drive.setPower(get_voltage_compensated(forward + turn));
    }
    public void intake_loop (){
        if (gamepad1.right_trigger_pressed)
            Intake.setPower(get_voltage_compensated(1));
        if (gamepad1.left_trigger_pressed)
            Intake.setPower(get_voltage_compensated(-1));
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
        set_velo_shooter(velo_shoot);
        viseur.setPosition(pos_viseur);
        //if (velo_shoot - shooter_tolerance < Shooter.getVelocity() && Shooter.getVelocity() < velo_shoot + shooter_tolerance)
          //  shooter_sys_state = Shooter_sys_state.READY;

    }
    public double get_power_to_auto_align (double angle_goal, int wanted_id){
        AprilTagDetection tag = null;
        for (AprilTagDetection detection : april_joke.getDetections()){
            if (detection.id == wanted_id)
                tag = detection;
        }
        if (tag == null){
            last_error = 0;
            timer_kd_rotation.reset();
            return 0;
        }

        else{
            error = angle_goal - tag.ftcPose.bearing;
            if (Math.abs(error) < tolerance_rotation){
                timer_kd_rotation.reset();
                last_error = 0;
                return 0;}
            double proportioal = kp_rotation*error;
            double derivative = ((error - last_error)/timer_kd_rotation.milliseconds())*kd_rotation;
            timer_kd_rotation.reset();
            last_error = error;
            return proportioal + derivative;
        }
    }
    @SuppressLint("DefaultLocale")
    public void telemetry(){
        telemetry.addData("velocité shooter (RPM)", get_Shooter_RPM());

        telemetry.addData("shooter kp", shooter_kp);
        telemetry.addData("shooter kv", shooter_kv);
        telemetry.addData("shooter ks", shooter_ks);
        telemetry.addData("shooter tolérance", shooter_tolerance);

        telemetry.addData("left stick x", gamepad1.left_stick_x);
        telemetry.addData("left stick y", gamepad1.left_stick_y);
        telemetry.addData("right stick x", gamepad1.right_stick_x);
        telemetry.addData("right stick y", gamepad1.right_stick_y);

        for (AprilTagDetection detection : april_joke.getDetections()) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        telemetry.update();
    }


    @Override
    public void init() {
        Init_motors();
        Init_camera();
        timer_kd_rotation.reset();
    }

    @Override
    public void loop() {
        voltage = voltageSensor.getVoltage();

        forward = gamepad1.left_stick_y;
        if (shooter_state == Shooter_wanted_state.Shoot_bank ||shooter_state == Shooter_wanted_state.Shoot_mid ||shooter_state == Shooter_wanted_state.Shoot_far ||){
            turn = gamepad1.right_stick_x + get_power_to_auto_align(0, 24);
        }else{
            turn = gamepad1.right_stick_x;
            timer_kd_rotation.reset();
            last_error = 0;
        }
        Drive_loop(forward, turn);

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
        telemetry();
    }
}
