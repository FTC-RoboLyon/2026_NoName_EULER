package org.firstinspires.ftc.teamcode.CommandBase.Command;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.Shooter.ShooterSubsystem;

public class SetShooterWantedState extends CommandBase {
    ShooterSubsystem shooterSubsystem;
    ShooterSubsystem.WantedState wantedState;

    public SetShooterWantedState(ShooterSubsystem shooter, ShooterSubsystem.WantedState wantedState){
        shooterSubsystem = shooter;
        this.wantedState = wantedState;
    }

    @Override
    public void initialize(){
        shooterSubsystem.setWantedState(wantedState);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
