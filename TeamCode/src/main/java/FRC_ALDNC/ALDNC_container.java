package FRC_ALDNC;



import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.arcrobotics.ftclib.command.button.Button;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import FRC_ALDNC.SubSystem.Drive_Train;
import FRC_ALDNC.SubSystem.Feeder_subsystem;
import FRC_ALDNC.SubSystem.Intake_subsystem;
import FRC_ALDNC.SubSystem.Shooter_Subsystem;
import FRC_ALDNC.SubSystem.joystick_subsystem;
import FRC_ALDNC.commands.Drive_command;
import FRC_ALDNC.commands.Let_a_ball_pass;
import FRC_ALDNC.commands.Shoot_a_ball_command;
import Webcam_aldnc_yeux.Apriltag_reader;

public class ALDNC_container{
    Drive_Train chassis_subsystem;
    Shooter_Subsystem shooter_subsystem;
    Intake_subsystem intake;
    Feeder_subsystem feeder;
    joystick_subsystem left_joystick;
    joystick_subsystem right_joystick;
    Telemetry telemetry;
    public enum RobotMode
    {
        AUTO_BLUE,
        AUTO_RED,
        TELEOP_RED,
        TELEOP_BLUE
    }
    public double m_voltageSensorValue;
    VoltageSensor voltageSensor;

    public ALDNC_container (HardwareMap hmap, RobotMode wich_programme, Gamepad gamepad, Telemetry telemetry){
        chassis_subsystem = new Drive_Train(hmap);
        shooter_subsystem = new Shooter_Subsystem(hmap);
        intake = new Intake_subsystem(hmap);
        feeder = new Feeder_subsystem(hmap);
        left_joystick = new joystick_subsystem(gamepad, joystick_subsystem.Witch_stick.left, chassis_subsystem);
        right_joystick = new joystick_subsystem(gamepad, joystick_subsystem.Witch_stick.right, chassis_subsystem);
        Apriltag_reader apriljoke = new Apriltag_reader(hmap);
        voltageSensor = hmap.get(VoltageSensor.class, "Control Hub");
        this.telemetry = telemetry;

        chassis_subsystem.setDefaultCommand(new Drive_command(chassis_subsystem, left_joystick, right_joystick, telemetry));

    }
    public void Configure_Binding(
            Button feeder_button,
            Button shoot_bank_button,
            Button shoot_mid_button,
            Button shoot_far_butto,
            Button intake_button){
        feeder_button.whenPressed(new Shoot_a_ball_command(shooter_subsystem, feeder));
        feeder_button.whenReleased(new Let_a_ball_pass(feeder));



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
