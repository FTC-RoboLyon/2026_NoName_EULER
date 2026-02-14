package FRC_ALDNC.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

import FRC_ALDNC.SubSystem.Drive_Train;

public class Drive_using_suplier_test extends CommandBase {
    private final Drive_Train chassis;
    private final DoubleSupplier forward;
    private final DoubleSupplier turn;
    public Drive_using_suplier_test (Drive_Train chassiS, DoubleSupplier forward, DoubleSupplier turn){
        chassis = chassiS;
        this.forward = forward;
        this.turn = turn;
    }

    @Override
    public void execute() {
        chassis.drive(forward.getAsDouble(), turn.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
