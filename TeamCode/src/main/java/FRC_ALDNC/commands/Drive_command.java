package FRC_ALDNC.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import FRC_ALDNC.SubSystem.Drive_Train;
import FRC_ALDNC.SubSystem.joystick_subsystem;

public class Drive_command extends CommandBase {
    Drive_Train chassi;
    ElapsedTime timer;
    double forward;
    double turn;
    double temps_drive;
    Telemetry telemetrY;

    public Drive_command (Drive_Train chassi, double forward, double turn, double temps_drive, Telemetry telemetry){
        telemetrY = telemetry;
        this.chassi = chassi;
        this.forward = forward;
        this.turn = turn;
        this.temps_drive = temps_drive;

        addRequirements(chassi);
    }
    @Override
    public void initialize(){
        chassi.drive(0, 0);
        timer = new ElapsedTime();
        timer.reset();
        telemetrY.addLine("drive command is initialize");
        telemetrY.update();
    }




    @Override
    public void execute() {
        chassi.drive(forward, turn);
        telemetrY.addLine("drive command is execute");
        telemetrY.update();
    }

    @Override
    public void end(boolean interrupted) {
        chassi.drive(0,0);
        telemetrY.addLine("drive command has end");
        telemetrY.update();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= temps_drive;
    }
}
