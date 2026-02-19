package FRC_ALDNC.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import FRC_ALDNC.SubSystem.Drive_Train;
import FRC_ALDNC.SubSystem.joystick_subsystem;

public class Drive_command extends CommandBase {
    Drive_Train chassi;
    joystick_subsystem stick_left;
    joystick_subsystem stick_right;

    Telemetry telemetry;
    public Drive_command (Drive_Train chassi, joystick_subsystem stick_left, joystick_subsystem stick_right, Telemetry telemetry){
        this.chassi = chassi;
        this.stick_left = stick_left;
        this.stick_right = stick_right;
        this.telemetry = telemetry;
        addRequirements(chassi, stick_left, stick_right);
    }
    @Override
    public void initialize(){}
    @Override
    public void execute(){
        chassi.drive(stick_left.getX(), stick_right.getY());
        telemetry.addLine("I am executing");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
