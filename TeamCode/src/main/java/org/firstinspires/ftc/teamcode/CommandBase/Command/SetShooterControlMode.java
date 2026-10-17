package org.firstinspires.ftc.teamcode.CommandBase.Command;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Lib.LyonLib.control.ControlMode;

public class SetShooterControlMode extends CommandBase {

    ShooterSubsystem shooterSubsystem;
    ControlMode controlMode;
    public SetShooterControlMode(ShooterSubsystem shooter, ControlMode ControlMode){
        shooterSubsystem = shooter;
        controlMode = ControlMode;
    }

    @Override
    public void initialize(){
        shooterSubsystem.setShooterControlMode(controlMode);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
