package FRC_ALDNC.Auto.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import FRC_ALDNC.Auto.Subsystems.intakeSubsystem;

public class intakeCommand extends CommandBase {
    intakeSubsystem intakeSubsystem;
    public enum intakage{OUI, NON, WAIT}
    intakage mode;
    public intakeCommand(intakeSubsystem intakeSubsystem, intakage mode){
        this.mode = mode;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }
    @Override
    public void execute() {
        switch (mode) {
            case OUI:
                intakeSubsystem.configureIntake("Intake");
                break;
            case NON:
                intakeSubsystem.configureIntake("Reject");
                break;
            case WAIT:
                intakeSubsystem.configureIntake("Wait");
                break;
        }
    }
}
