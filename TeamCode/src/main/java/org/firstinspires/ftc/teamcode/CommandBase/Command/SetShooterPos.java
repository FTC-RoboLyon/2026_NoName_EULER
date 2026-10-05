package org.firstinspires.ftc.teamcode.CommandBase.Command;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.ShooterSubsystem;

public class SetShooterPos extends CommandBase {
    //Dcp c pas vrm set shooter Pos mais juste setShooterWantedStateCmd
    ShooterSubsystem shooterSubsystem;
    ShooterSubsystem.WantedState wantedState;

    public SetShooterPos(ShooterSubsystem shooter, ShooterSubsystem.WantedState wantedState){
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
