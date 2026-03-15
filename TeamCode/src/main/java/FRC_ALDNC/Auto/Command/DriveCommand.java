package FRC_ALDNC.Auto.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import FRC_ALDNC.Auto.Subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
    private double x, y;
    String variableSwitch;
    Gamepad gamepad1;
    Telemetry telemetry;
    DriveSubsystem driveSubsystem;
    public enum driveMode{ GoPos, GoAngle }
    driveMode mode;
    public DriveCommand(DriveSubsystem driveSubsystem, Gamepad gamepad1, Telemetry telemetry, driveMode mode, double x, double y) {
        this.x = x;
        this.y = y;
        this.mode = mode;
        this.telemetry = telemetry;
        this.driveSubsystem = driveSubsystem;
        this.gamepad1 = gamepad1;
        addRequirements(driveSubsystem);
    }
    @Override
    public void execute(){
        if (gamepad1.xWasPressed()){
            x -= 1;
        }else if(gamepad1.bWasPressed()){
            x += 1;
        }else if(gamepad1.yWasPressed()){
            y += 1;
        }else if(gamepad1.aWasPressed()){
            y -= 1;
        }
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
          return driveSubsystem.getAngleTo() < 0.01;
      }else {
          return driveSubsystem.getDistanceTo(x,y) < 1;
      }
    }

    @Override
    public void end(boolean interrupted){
        driveSubsystem.right_motor_power = 0;
        driveSubsystem.left_motor_power = 0;
    }
}
