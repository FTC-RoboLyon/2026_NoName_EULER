package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "TestBasetankAntoine1fordistance1 (Blocks to Java)")
public class TestBasetankAntoine1fordistance1 extends LinearOpMode {

  private DcMotor leftmotor;

  private CRServo Intake;
  private DcMotor shooterright;
  private DcMotor shooterleft;
  private IMU imu;
  private DistanceSensor distancesensor;
  private DcMotor rightmotor;
  private CRServo feeder;

  double distance_to_the_goal;
  float Turn;
  int CPR;
  double robotOrienRadian;
  double robotOrienDegrees;

  int velocity_motor;
  float Forward;
  int distancce_to_goal_parfaite_pour_tirer;
  double circumference;
  int robot_orientation_parfaite;
  float distance_between_wheels;
  double robot_orientation_with_encoder;

  /**
   * Describe this function...
   */
  private void inizialisation() {
    leftmotor.setDirection(DcMotor.Direction.REVERSE);
    Intake.setDirection(CRServo.Direction.REVERSE);
    shooterright.setDirection(DcMotor.Direction.REVERSE);
    shooterleft.setDirection(DcMotor.Direction.FORWARD);
    shooterleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    shooterright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    velocity_motor = 1000;
    // Remplacer les 0 par les valeurs voulues
    distancce_to_goal_parfaite_pour_tirer = 0;
    robot_orientation_parfaite = 52;
    distance_between_wheels = 14.5F;
    robot_orientation_with_encoder = 0;
  }

  /**
   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue Comment
   * Blocks show where to place Initialization code (runs once, after touching the DS INIT
   * button, and before touching the DS Start arrow), Run code (runs once, after touching
   * Start), and Loop code (runs repeatedly while the OpMode is active, namely not Stopped).
   */
  @Override
  public void runOpMode() {
    leftmotor = hardwareMap.get(DcMotor.class, "left motor");
    Intake = hardwareMap.get(CRServo.class, "Intake");
    shooterright = hardwareMap.get(DcMotor.class, "shooter right");
    shooterleft = hardwareMap.get(DcMotor.class, "shooter left");
    imu = hardwareMap.get(IMU.class, "imu");
    distancesensor = hardwareMap.get(DistanceSensor.class, "distance sensor");
    rightmotor = hardwareMap.get(DcMotor.class, "right motor");
    feeder = hardwareMap.get(CRServo.class, "feeder");

    // Put initialization blocks here.
    inizialisation();
    imu_inizialisation();
    encoder_initilization();
    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        // Put loop blocks here.
        telemetry2();
        drive();
        intake();
        Shooter();
        se_positioner();
        }
      }
    }

    /**
     * Describe this function...
     */
    private void imu_inizialisation() {
      IMU.Parameters imu_parameters;

      // Create a RevHubOrientationOnRobot object for use with an IMU in a REV Robotics
      // Control Hub or Expansion Hub, specifying the hub's arbitrary orientation on
      // the robot via an Orientation block that describes the rotation that would
      // need to be applied in order to rotate the hub from having its logo facing up1
      // and the USB ports facing forward, to its actual orientation on the robot.
      imu_parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.xyzOrientation(0, 0, 0)));
      imu.initialize(imu_parameters);
      imu.resetYaw();
    }

    /**
     * Describe this function... initialize 
     */
    public void encoder_initilization() {
      int diameter;

      CPR = 560;
      diameter = 9;
      circumference = Math.PI * diameter;
    }

    /**
     * Describe this function...
     */
    public void telemetry2() {
      savoir_l_orientation();
      savoir_la_distance();
      encoders();
      telemetry.update();
    }

    /**
     * dfcfcfcf
     */
    public void savoir_l_orientation() {
      YawPitchRollAngles robotYawPitchRoll;

      robotYawPitchRoll = imu.getRobotYawPitchRollAngles();
      robotOrienRadian = -robotYawPitchRoll.getYaw(AngleUnit.RADIANS);
      robotOrienDegrees = -robotYawPitchRoll.getYaw(AngleUnit.DEGREES);
      telemetry.addData("robot orientation", robotOrienRadian);
    }

    /**
     * Describe this function...
     */
    public void savoir_la_distance() {
      distance_to_the_goal = distancesensor.getDistance(DistanceUnit.CM);
      telemetry.addData("distance from the goal ", distance_to_the_goal);
    }

    /**
     * Describe this function...
     */
    public void encoders() {
      int right_wheels_ticks;
      int left_wheel_ticks;
      int left_wheel_nb_of_revolution;
      int right_wheels_nb_of_revolution;
      double distance_parcouru_right;
      double distance_parcouru_left;

      right_wheels_ticks = rightmotor.getCurrentPosition();
      left_wheel_ticks = leftmotor.getCurrentPosition();
      left_wheel_nb_of_revolution = left_wheel_ticks / CPR;
      right_wheels_nb_of_revolution = right_wheels_ticks / CPR;
      distance_parcouru_right = circumference * right_wheels_nb_of_revolution;
      distance_parcouru_left = circumference * left_wheel_nb_of_revolution;
      telemetry.addData("distance parcouru left ", distance_parcouru_left);
      telemetry.addData("distance parcouru right", distance_parcouru_right);
      telemetry.addLine("antoine le bg du 69");
    }

    /**
     * Describe this function...
     */
    private void se_positioner() {
      if (gamepad1.right_trigger > 0.1) {
        while (opModeIsActive()) {
          leftmotor.setPower(-0.5);
          rightmotor.setPower(0.5);
          savoir_l_orientation();
          if (robot_orientation_parfaite - 5 <= -robotOrienDegrees && robot_orientation_parfaite <= robotOrienDegrees + 5){
            leftmotor.setPower(0);
            rightmotor.setPower(0);
            break;
          }
        }

        while (opModeIsActive()) {
          if (distance_to_the_goal > distancce_to_goal_parfaite_pour_tirer) {
            leftmotor.setPower(0.5);
            rightmotor.setPower(0.5);
          } else if (distance_to_the_goal < distancce_to_goal_parfaite_pour_tirer) {
            leftmotor.setPower(-0.5);
            rightmotor.setPower(-0.5);
          } else {
            leftmotor.setPower(0);
            rightmotor.setPower(0);
          }
          savoir_la_distance();
          if (distancce_to_goal_parfaite_pour_tirer - 0.1 <= distance_to_the_goal && distance_to_the_goal <= distancce_to_goal_parfaite_pour_tirer + 0.1){
            leftmotor.setPower(0);
            rightmotor.setPower(0);
            break;
          }
        }
      }
    }

    /**
     * Describe this function...
     */
    public void Shooter() {
      if (gamepad1.dpad_up) {
        velocity_motor += 50;
      } else if (gamepad1.dpad_down) {
        velocity_motor += -50;
      } else if (gamepad1.a) {
        ((DcMotorEx) shooterleft).setVelocity(velocity_motor);
        ((DcMotorEx) shooterright).setVelocity(velocity_motor);
      } else if (gamepad1.b) {
        feeder.setPower(1);
      } else if (gamepad1.x) {
        feeder.setPower(-1);
      } else {
        ((DcMotorEx) shooterleft).setVelocity(0);
        ((DcMotorEx) shooterright).setVelocity(0);
        feeder.setPower(0);
      }
    }

    /**
     * Describe this function...
     */
    private void drive() {
      Turn = gamepad1.right_stick_x;
      if (-90 <= robotOrienDegrees && robotOrienDegrees <= 90) {
        Forward = -gamepad1.left_stick_y;
      }
      else {
        Forward = gamepad1.left_stick_y;
      }
      leftmotor.setPower(Forward + Turn);
      rightmotor.setPower(Forward - Turn);
    }

    /**
     * Describe this function...
     */
    private void intake() {
      if (gamepad1.left_bumper) {
        Intake.setPower(1);
      } else if (gamepad1.right_bumper) {
        Intake.setPower(-1);
      } else {
        Intake.setPower(0);
      }
    }
}


/*
Dans cette section, je code une fonction pour piloter une base mecanum en fonction de l'orientation du robot. N'ayant pas assez de moteurs et de port pour l'instant, je crée cette fonction en commentaire :

float strafe;             a rajouter dans l'initilisation des variables
public void mecanumDrive() {
    Turn = gamepad.right_stick_x
    Forward = gamepade.left_stick_y * robotOrienDegrees
    strafe =
    FrontLeft.setPower(Forward + strafe + Turn );
    FrontRight.setPower(Forward - strafe - Turn);
    BackLeft.setPower(Forward - starfe + Turn);
    BackRight.setPower(Forward + starfe - Turn);
 */

