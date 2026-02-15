package FRC_ALDNC;



//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.arcrobotics.ftclib.command.button.Button;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

import FRC_ALDNC.SubSystem.Camera_subsystem;
import FRC_ALDNC.SubSystem.Drive_Train;
import FRC_ALDNC.SubSystem.Feeder_subsystem;
import FRC_ALDNC.SubSystem.Intake_subsystem;
import FRC_ALDNC.SubSystem.Shooter_Subsystem;
import FRC_ALDNC.SubSystem.joystick_subsystem;
import FRC_ALDNC.commands.Drive_command;
import FRC_ALDNC.commands.Drive_using_suplier_test;
import FRC_ALDNC.commands.Let_a_ball_pass;
import FRC_ALDNC.commands.Shoot_a_ball_command;
import FRC_ALDNC.commands.Collect_command;
import FRC_ALDNC.commands.Configure_shooter;
import FRC_ALDNC.commands.Tuning_postir_command;


public class ALDNC_container{
    Drive_Train chassis_subsystem;
    Shooter_Subsystem shooter_subsystem;
    Intake_subsystem intake;
    Feeder_subsystem feeder;
    joystick_subsystem left_joystick;
    joystick_subsystem right_joystick;
    Telemetry telemetry;
    Camera_subsystem apriljoke;
    DoubleSupplier forward;
    DoubleSupplier turn;
    public enum RobotMode
    {
        AUTO_BLUE,
        AUTO_RED,
        TELEOP_RED,
        TELEOP_BLUE
    }

    RobotMode team_and_mode;
    public double m_voltageSensorValue;
    VoltageSensor voltageSensor;

    public ALDNC_container (HardwareMap hmap, RobotMode wich_programme, GamepadEx gamepad, Telemetry telemetry){
        chassis_subsystem = new Drive_Train(hmap);

        shooter_subsystem = new Shooter_Subsystem(hmap);

        intake = new Intake_subsystem(hmap);

        feeder = new Feeder_subsystem(hmap);

        left_joystick = new joystick_subsystem(gamepad, joystick_subsystem.Witch_stick.left, chassis_subsystem);
        right_joystick = new joystick_subsystem(gamepad, joystick_subsystem.Witch_stick.right, chassis_subsystem);
        //forward = () -> left_joystick.getX();
        //turn = () -> right_joystick.getY();    Si le chassis ne bouge pas avec le programme actuel, ajouter les lignes 66, 67 et 77 et enlevez la ligne 75

        apriljoke = new Camera_subsystem(hmap, wich_programme == RobotMode.TELEOP_RED || wich_programme == RobotMode.AUTO_RED ? 24 : 20);

        voltageSensor = hmap.get(VoltageSensor.class, "Control Hub");

        team_and_mode = wich_programme;

        chassis_subsystem.setDefaultCommand(new Drive_command(chassis_subsystem, left_joystick, right_joystick, telemetry));
        shooter_subsystem.setDefaultCommand(new Tuning_postir_command(shooter_subsystem, apriljoke));
        //chassis_subsystem.setDefaultCommand(new Drive_using_suplier_test(chassis_subsystem, forward, turn));

        this.telemetry = telemetry;
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    }
    public void Configure_Binding(
            Button feeder_button,
            Button shoot_bank_button,
            Button shoot_mid_button,
            Button shoot_far_butto,
            Button aspirer_button,
            Button intake_button,
            Trigger eject_button){

        feeder_button.whenPressed(new Shoot_a_ball_command(shooter_subsystem, feeder));
        feeder_button.whenReleased(new Let_a_ball_pass(feeder));

        intake_button.whenPressed(new Collect_command(intake, Intake_subsystem.WantedState.COLLECT));
        intake_button.whenReleased(new Collect_command(intake, Intake_subsystem.WantedState.STAND_BY));

        shoot_bank_button.whenPressed(new Configure_shooter(shooter_subsystem, Shooter_Subsystem.WantedState.SHOOT_BANK));
        shoot_bank_button.whenReleased(new Configure_shooter(shooter_subsystem, Shooter_Subsystem.WantedState.WAIT));

        shoot_mid_button.whenPressed(new Configure_shooter(shooter_subsystem, Shooter_Subsystem.WantedState.SHOOT_MID));
        shoot_mid_button.whenReleased(new Configure_shooter(shooter_subsystem, Shooter_Subsystem.WantedState.WAIT));

        shoot_far_butto.whenPressed(new Configure_shooter(shooter_subsystem, Shooter_Subsystem.WantedState.SHOOT_FAR));
        shoot_far_butto.whenReleased(new Configure_shooter(shooter_subsystem, Shooter_Subsystem.WantedState.WAIT));

        aspirer_button.whenPressed(new Configure_shooter(shooter_subsystem, Shooter_Subsystem.WantedState.ASPIRER));
        aspirer_button.whenPressed(new Configure_shooter(shooter_subsystem, Shooter_Subsystem.WantedState.WAIT));

        eject_button.whenActive(new Collect_command(intake, Intake_subsystem.WantedState.EJECT));
        eject_button.whenActive(new Collect_command(intake, Intake_subsystem.WantedState.STAND_BY));
    }
    public void telemetry (){
        telemetry.addData("Vrai vélocité shooter", shooter_subsystem.shooter.getVelocity());
        telemetry.addData("vélocité programmé shooter", shooter_subsystem.getVeloShooter());
        telemetry.addData("angle viseur", shooter_subsystem.viseur.getPosition());
        telemetry.addData("joystick_gauche", left_joystick.getX());
        telemetry.addData("joystick_droit", right_joystick.getY());
        apriljoke.telemetry(telemetry);
        telemetry.update();
    }
    public void ActualiseVoltageSensorValue()
    {
        m_voltageSensorValue = voltageSensor.getVoltage();
    }

    public double GetVoltageSensorValue()
    {
        return m_voltageSensorValue;
    }

}
