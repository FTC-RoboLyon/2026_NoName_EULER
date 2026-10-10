package org.firstinspires.ftc.teamcode.CommandBase.Command;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    IntakeSubsystem intakeSubsystem;
    IntakeSubsystem.IntakeWanteState intakeMode;

    public IntakeCommand(IntakeSubsystem intake, IntakeSubsystem.IntakeWanteState intakeMode){
        intakeSubsystem = intake;
        this.intakeMode = intakeMode;
    }

    @Override
    public void initialize(){
        intakeSubsystem.setIntakeWanteState(intakeMode);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
