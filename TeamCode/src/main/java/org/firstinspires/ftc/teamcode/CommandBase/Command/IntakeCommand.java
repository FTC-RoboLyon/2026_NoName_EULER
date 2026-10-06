package org.firstinspires.ftc.teamcode.CommandBase.Command;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.ShooterSubsystem;

public class IntakeCommand extends CommandBase {
    IntakeSubsystem intakeSubsystem;
    IntakeSubsystem.IntakeMode intakeMode;

    public IntakeCommand(IntakeSubsystem intake, IntakeSubsystem.IntakeMode intakeMode){
        intakeSubsystem = intake;
        this.intakeMode = intakeMode;
    }

    @Override
    public void initialize(){
        intakeSubsystem.setIntakeMode(intakeMode);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
