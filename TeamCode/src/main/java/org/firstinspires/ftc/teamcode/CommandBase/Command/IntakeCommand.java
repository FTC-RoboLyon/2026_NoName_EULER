package org.firstinspires.ftc.teamcode.CommandBase.Command;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    IntakeSubsystem intakeSubsystem;
    IntakeSubsystem.IntakeWanteState intakeMode;

    public IntakeCommand(IntakeSubsystem intake, IntakeSubsystem.IntakeWanteState intakeMode){ //IntakeCommand ne veut rien dire, il faut que son nom dise a quoi elle sert, en l'occurence changer le wanted state de l'intake)
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
