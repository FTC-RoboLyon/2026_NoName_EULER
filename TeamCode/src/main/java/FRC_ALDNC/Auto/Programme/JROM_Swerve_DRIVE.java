package FRC_ALDNC.Auto.Programme;


import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "JROM_Swerve_DRIVE (Blocks to Java)")
public class JROM_Swerve_DRIVE extends LinearOpMode {

    private VoltageSensor ControlHub_VoltageSensor;
    private DcMotor LeftBottomMotor_and_LeftDeadWheel;
    private DcMotor LeftTopMotor;
    private DcMotor RightBottomMotor_and_RightDeadWheel;
    private DcMotor RightTopMotor;
    private AnalogInput LeftHallEffectSensor;
    private AnalogInput RightHallEffectSensor;
    private DcMotor roue_morte_straffeAsDcMotor;

    int LeftEncoderAimingPosition;
    int RightEncoderAimingPosition;
    double LeftMaxHallEffect;
    double RightMaxHallEffect;
    private NavxMicroNavigationSensor Navx;

    /**
     * Describe this function...
     */
    private double GetVoltageConpensed(double Consigne) {
        double C;

        C = (Consigne * ControlHub_VoltageSensor.getVoltage()) / 12;
        return C;
    }

    /**
     * Describe this function...
     */
    private void motors_setup() {
        LeftBottomMotor_and_LeftDeadWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftTopMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBottomMotor_and_RightDeadWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightTopMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBottomMotor_and_LeftDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftTopMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBottomMotor_and_RightDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightTopMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBottomMotor_and_LeftDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftTopMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBottomMotor_and_RightDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightTopMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Describe this function...
     */
    private void initialization() {
        motors_setup();
        IMU_setup();
        swerve_setup();
        telemetry.addLine("Touch the \"Touchpad\" to start calibrating the swerves");
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void swerve_setup() {
        // Initialize the NavX Micro sensor. This method must be called before using the other methods.
        Navx.initialize();
        // Initialisation des modules Swerves RBL.
        Swerve_RBL.InitSwerve();
        // Initialisation des moteurs des modules Swerves RBL.
        Swerve_RBL.HardwareSetup("LeftTopMotor", "LeftBottomMotor_and_LeftDeadWheel", "RightTopMotor", "RightBottomMotor_and_RightDeadWheel", "LeftHallEffectSensor", "RightHallEffectSensor");
        // Définir la geometrie de la base swerve.
        Swerve_RBL.robotGeometrySetup(0.21566, 0.1621, 0.055, 0.0254, 0.0254, 0.016);
        // Initialisation des capteurs de localisation de la base Swerve RBL.
        Swerve_RBL.localizationSensorsSetup("navx", "roue_morte_gauche", "roue_morte_droite", "roue_morte_straffe", true, false, true, 8192, 8192, 2000);
        // Initialisation des Constantes des modules Swerves RBL.
        Swerve_RBL.ConstantsSetup(0.1, 10);
        // Divers parametres.
        Swerve_RBL.miscParametersSetup(1, 0.01, 0.01, 2, 0, 0.001, 0.9, 0, 0.001);
        // Initialisation des Constantes des modules Swerves RBL.
        Swerve_RBL.SwerveModuleSetup(8192 * 3, 1, 2, 0.001 * 0.02, 0.05 / 0.02, 0);
        LeftEncoderAimingPosition = 0;
        RightEncoderAimingPosition = 0;
        LeftMaxHallEffect = 0;
        RightMaxHallEffect = 0;
        // Initialisation PID
        Pid_RBL.Init(8, 0.00045, 0, 0, 5);
        // Initialisation PID
        Pid_RBL.Init(9, 0.00065, 0, 0, 5);
    }

    /**
     * Describe this function...
     */
    private void Calibration() {
        ElapsedTime time2;
        double LeftHallEffect;
        double LeftBottomMotorPower;
        double PID_Left;
        double LeftTopMotorPower;
        double RightBottomMotorPower;
        double RightTopMotorPower;
        double RightHallEffect;
        double PID_Right;
        double max;

        while (!isStarted()) {
            if (gamepad1.touchpad) {
                time2 = new ElapsedTime();
                while (!isStarted()) {
                    if (LeftTopMotor.getCurrentPosition() >= 8192 * -3.5) {
                        LeftHallEffect = LeftHallEffectSensor.getVoltage();
                        LeftBottomMotorPower = GetVoltageConpensed(0.1);
                        LeftTopMotorPower = GetVoltageConpensed(0.1);
                        if (LeftMaxHallEffect < LeftHallEffect) {
                            LeftMaxHallEffect = LeftHallEffect;
                            LeftEncoderAimingPosition = LeftTopMotor.getCurrentPosition() + 4096 * -3;
                        }
                    } else {
                        LeftBottomMotorPower = GetVoltageConpensed(0);
                        LeftTopMotorPower = GetVoltageConpensed(0);
                    }
                    if (RightTopMotor.getCurrentPosition() >= 8192 * -3.5) {
                        RightBottomMotorPower = GetVoltageConpensed(0.15);
                        RightTopMotorPower = GetVoltageConpensed(0.15);
                        RightHallEffect = RightHallEffectSensor.getVoltage();
                        if (RightMaxHallEffect < RightHallEffect) {
                            RightMaxHallEffect = RightHallEffect;
                            RightEncoderAimingPosition = RightTopMotor.getCurrentPosition() + 4096 * -3;
                        }
                    } else {
                        RightBottomMotorPower = GetVoltageConpensed(0);
                        RightTopMotorPower = GetVoltageConpensed(0);
                    }
                    if (RightBottomMotorPower == 0 && LeftBottomMotorPower == 0) {
                        break;
                    }
                    LeftBottomMotor_and_LeftDeadWheel.setPower(LeftBottomMotorPower);
                    LeftTopMotor.setPower(-LeftTopMotorPower);
                    RightBottomMotor_and_RightDeadWheel.setPower(RightBottomMotorPower);
                    RightTopMotor.setPower(-RightTopMotorPower);
                    telemetry.addLine("Left Encoder Position : " + LeftTopMotor.getCurrentPosition());
                    telemetry.addLine("Left Encoder Max Position : " + LeftEncoderAimingPosition);
                    telemetry.addLine("Right Encoder Position : " + RightTopMotor.getCurrentPosition());
                    telemetry.addLine("Right Encoder Max Position : " + RightEncoderAimingPosition);
                    telemetry.addLine("PID Left : " + PID_Left);
                    telemetry.addLine("PID Right : " + PID_Right);
                    telemetry.update();
                }
                while (!isStarted()) {
                    // calcul du PID avec tolérance
                    PID_Left = -Pid_RBL.CalculateWithTolerance(8, LeftEncoderAimingPosition, LeftTopMotor.getCurrentPosition());
                    LeftBottomMotorPower = GetVoltageConpensed(PID_Left);
                    LeftTopMotorPower = GetVoltageConpensed(PID_Left);
                    // calcul du PID avec tolérance
                    PID_Right = -Pid_RBL.CalculateWithTolerance(9, RightEncoderAimingPosition, RightTopMotor.getCurrentPosition());
                    RightBottomMotorPower = GetVoltageConpensed(PID_Right);
                    RightTopMotorPower = GetVoltageConpensed(PID_Right);
                    // calcul du PID avec tolérance
                    // calcul du PID avec tolérance
                    if (Pid_RBL.CalculateWithTolerance(8, LeftEncoderAimingPosition, LeftTopMotor.getCurrentPosition()) == 0 && Pid_RBL.CalculateWithTolerance(9, RightEncoderAimingPosition, RightTopMotor.getCurrentPosition()) == 0) {
                        LeftBottomMotor_and_LeftDeadWheel.setPower(0);
                        LeftTopMotor.setPower(0);
                        RightBottomMotor_and_RightDeadWheel.setPower(0);
                        RightTopMotor.setPower(0);
                        motors_setup();
                        break;
                    }
                    max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(LeftBottomMotorPower), Math.abs(LeftTopMotorPower), Math.abs(RightBottomMotorPower), Math.abs(RightTopMotorPower)));
                    if (max > 1) {
                        LeftBottomMotorPower = LeftBottomMotorPower / max;
                        LeftTopMotorPower = LeftTopMotorPower / max;
                        RightBottomMotorPower = RightBottomMotorPower / max;
                        RightTopMotorPower = RightTopMotorPower / max;
                    }
                    if (15000 <= time2.milliseconds()) {
                        break;
                    }
                    LeftBottomMotor_and_LeftDeadWheel.setPower(LeftBottomMotorPower);
                    LeftTopMotor.setPower(-LeftTopMotorPower);
                    RightBottomMotor_and_RightDeadWheel.setPower(RightBottomMotorPower);
                    RightTopMotor.setPower(-RightTopMotorPower);
                    telemetry.addLine("Left Encoder Position : " + LeftTopMotor.getCurrentPosition());
                    telemetry.addLine("Left Encoder Max Position : " + LeftEncoderAimingPosition);
                    telemetry.update();
                }
                telemetry.addLine("Calibration complete !!!");
                telemetry.update();
                break;
            }
        }
    }

    /**
     * Describe this function...
     */
    @Override
    public void runOpMode() {
        float LeftJoystickX;
        float LeftJoystickY;
        float RightJoystickX;
        int autoStep;

        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        LeftBottomMotor_and_LeftDeadWheel = hardwareMap.get(DcMotor.class, "LeftBottomMotor_and_LeftDeadWheelAsDcMotor");
        LeftTopMotor = hardwareMap.get(DcMotor.class, "LeftTopMotorAsDcMotor");
        RightBottomMotor_and_RightDeadWheel = hardwareMap.get(DcMotor.class, "RightBottomMotor_and_RightDeadWheelAsDcMotor");
        RightTopMotor = hardwareMap.get(DcMotor.class, "RightTopMotorAsDcMotor");
        LeftHallEffectSensor = hardwareMap.get(AnalogInput.class, "LeftHallEffectSensorAsAnalogInput");
        RightHallEffectSensor = hardwareMap.get(AnalogInput.class, "RightHallEffectSensorAsAnalogInput");
        roue_morte_straffeAsDcMotor = hardwareMap.get(DcMotor.class, "roue_morte_straffeAsDcMotor");

        initialization();
        Calibration();
        motors_setup();
        // It resets the Heading (or Yaw or Z-angle) to zero. It's
        // safest to reset the Yaw when the robot is on a flat surface.
        Gyro_RBL.Reset();
        // Resets the yaw angle to zero by setting an offset.
        Navx.resetYaw();
        LeftBottomMotor_and_LeftDeadWheel.setPower(0);
        LeftTopMotor.setPower(0);
        RightBottomMotor_and_RightDeadWheel.setPower(0);
        RightTopMotor.setPower(0);
        while (opModeInInit()) {
        }
        waitForStart();
        // reset l'origine de reference du déplacement autonome.
        Swerve_RBL.resetZero();
        autoStep = 0;
        // Bloc de deplacement autonome.
        Swerve_RBL.setAutonomousParameters(1, 0, Math.PI / -2, 0.3, 3);
        while (opModeIsActive()) {
            LeftJoystickY = gamepad1.left_stick_y;
            LeftJoystickX = gamepad1.left_stick_x;
            RightJoystickX = gamepad1.right_stick_x;
            // DRIVE !!!
            // Calcule de la direction des modules swerves
            Swerve_RBL.Drive2(LeftJoystickX, LeftJoystickY, RightJoystickX);
        }
    }

    /**
     * Describe this function...
     */
    private void IMU_setup() {
        // Initialize the IMU object. This method must be called before using the other methods.
        Gyro_RBL.InitIMU();
        // Link the IMU object to the hardware. This method must be called before using the other methods.
        Gyro_RBL.HardwareSetup("imu");
        // Initialization of the starting position of the IMU. This method must be called
        // to set the initial position of the non-orthoganal IMU before using them.
        Gyro_RBL.SetStartingPosition(0, 0, 0, AngleUnit.DEGREES);
    }
}

