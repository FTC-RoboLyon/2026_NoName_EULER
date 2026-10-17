package org.firstinspires.ftc.teamcode.CommandBase.Command;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.ShooterSubsystem;

public class SetIntakeControlMode extends CommandBase {

    IntakeSubsystem intakeSubsystem;
    IntakeSubsystem.ControlMode controlMode;

    public SetIntakeControlMode(IntakeSubsystem intakeSubsystem, IntakeSubsystem.ControlMode controlMode){
        this.intakeSubsystem = intakeSubsystem;
        this.controlMode = controlMode;
    }
    @Override
    public void initialize(){
        intakeSubsystem.setIntakeControlMode(controlMode);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
