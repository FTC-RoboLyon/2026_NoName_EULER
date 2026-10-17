package org.firstinspires.ftc.teamcode.CommandBase.Command;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.IntakeSubsystem;

public class SetIntakeWantedState extends CommandBase {
    IntakeSubsystem intakeSubsystem;
    IntakeSubsystem.WanteState intakeMode;

    public SetIntakeWantedState(IntakeSubsystem intake, IntakeSubsystem.WanteState intakeMode){
        intakeSubsystem = intake;
        this.intakeMode = intakeMode;
    }

    @Override
    public void initialize(){
        intakeSubsystem.setIntakeWantedState(intakeMode);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
