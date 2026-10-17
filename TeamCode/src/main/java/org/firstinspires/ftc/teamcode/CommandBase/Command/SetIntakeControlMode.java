package org.firstinspires.ftc.teamcode.CommandBase.Command;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Lib.LyonLib.control.ControlMode;

public class SetIntakeControlMode extends CommandBase {

    IntakeSubsystem intakeSubsystem;
    ControlMode controlMode;

    public SetIntakeControlMode(IntakeSubsystem intakeSubsystem, ControlMode controlMode)
    {
        this.intakeSubsystem = intakeSubsystem;
        this.controlMode = controlMode;
    }
    @Override
    public void initialize(){
        intakeSubsystem.setControlMode(controlMode);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
