package FRC_ALDNC.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import FRC_ALDNC.SubSystem.Intake_subsystem;

public class Collect_command extends CommandBase {
    Intake_subsystem intake_subsystem;
    Intake_subsystem.WantedState go_or_not;
    public Collect_command(Intake_subsystem intake, Intake_subsystem.WantedState to_go_or_not_to_go){
        go_or_not = to_go_or_not_to_go;
        intake_subsystem = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake_subsystem.set_wantedState(go_or_not);
    }

    @Override
    public boolean isFinished() {
        return  true;
    }
}
