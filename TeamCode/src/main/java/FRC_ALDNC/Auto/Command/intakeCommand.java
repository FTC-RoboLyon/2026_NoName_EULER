package FRC_ALDNC.Auto.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import FRC_ALDNC.Auto.Subsystems.intakeSubsystem;

public class intakeCommand extends CommandBase {
    intakeSubsystem intakeSubsystem;
    Gamepad gamepad1;
    enum intakage{OUI, NON, WAIT}
    intakage mode;
    public intakeCommand(intakeSubsystem intakeSubsystem, Gamepad gamepad1){
        this.intakeSubsystem = intakeSubsystem;
        this.gamepad1 = gamepad1;
        addRequirements(intakeSubsystem);
    }
    @Override
    public void execute() {
        if(gamepad1.rightBumperWasPressed()){
            mode = intakage.OUI;
        }else if(gamepad1.right_trigger > 0.3){
            mode = intakage.NON;
        }else {
            mode = intakage.WAIT;
        }
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
