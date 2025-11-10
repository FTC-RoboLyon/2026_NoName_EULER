package org.firstinspires.ftc.teamcode;


public class code_main extends LinearOpMode {

    protected void importation() {
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

    }

    protected void création () {

        private DcMotor leftmotor;
        private DcMotor rightmotor;
        private CRServo Intake;
        private DcMotor shooterright;
        private DcMotor shooterleft;
        private IMU imu;
        private DistanceSensor distancesensor;
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
        float x;
        float y;
    }

    //Si vous utilisez d'autres variables dans vos codes mettez les là


    protected void inizialisation() {
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


    protected void imu_inizialisation() {
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

    protected void encoder_inizilization() {
        int diameter;

        CPR = 560;
        diameter = 9;
        circumference = Math.PI * diameter;
    }

    protected void nommage() {
        leftmotor = hardwareMap.get(DcMotor.class, "left motor");
        Intake = hardwareMap.get(CRServo.class, "Intake");
        shooterright = hardwareMap.get(DcMotor.class, "shooter right");
        shooterleft = hardwareMap.get(DcMotor.class, "shooter left");
        imu = hardwareMap.get(IMU.class, "imu");
        distancesensor = hardwareMap.get(DistanceSensor.class, "distance sensor");
        rightmotor = hardwareMap.get(DcMotor.class, "right motor");
        feeder = hardwareMap.get(CRServo.class, "feeder");


    }
}