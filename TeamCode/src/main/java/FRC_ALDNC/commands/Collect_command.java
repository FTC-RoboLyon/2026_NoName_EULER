package FRC_ALDNC.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.robocol.Command;

import FRC_ALDNC.SubSystem.Intake_subsystem;

public class Collect_command extends CommandBase {
    Intake_subsystem intake;
    public Collect_command (Intake_subsystem intake){
        this.intake = intake;
        addRequirements(intake);
    }
    @Override
    public void initialize(){
        intake.set_wantedState(Intake_subsystem.WantedState.COLLECT);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
