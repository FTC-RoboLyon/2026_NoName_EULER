package FRC_ALDNC.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;

public class Wait_for_start extends CommandBase {
    BooleanSupplier wait;
    public Wait_for_start (BooleanSupplier Wait){
        wait = Wait;
    }

    @Override
    public void initialize() {
        return;
    }

    @Override
    public boolean isFinished() {
        return wait.getAsBoolean();
    }
}
