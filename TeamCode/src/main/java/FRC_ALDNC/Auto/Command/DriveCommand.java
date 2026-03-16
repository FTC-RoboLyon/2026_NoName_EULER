package FRC_ALDNC.Auto.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import FRC_ALDNC.Auto.Subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
    private double x, y;
    Telemetry telemetry;
    DriveSubsystem driveSubsystem;
    public enum driveMode{ GoPos, GoAngle }
    driveMode mode;
    public DriveCommand(DriveSubsystem driveSubsystem, Telemetry telemetry, driveMode mode, double x, double y) {
        this.x = x;
        this.y = y;
        this.mode = mode;
        this.telemetry = telemetry;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }
    @Override
    public void execute(){
        switch(mode){
            case GoPos:
                driveSubsystem.goPos(x,y);
                break;
            case GoAngle:
                driveSubsystem.goAngle(x,y);
                break;
        }
        telemetry.addData("targetX", x);
        telemetry.addData("targetY", y);

    }
    @Override
    public boolean isFinished(){
      if(mode == driveMode.GoAngle){
          return driveSubsystem.getAngleTo() < 0.02;
      }else {
          return driveSubsystem.getDistanceTo(x,y) < 1.5;
      }
    }

    @Override
    public void end(boolean interrupted){
        driveSubsystem.right_motor_power = 0;
        driveSubsystem.left_motor_power = 0;
    }
}
