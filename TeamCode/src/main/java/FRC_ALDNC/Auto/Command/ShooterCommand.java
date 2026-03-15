package FRC_ALDNC.Auto.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.ams.AMSColorSensor;

import FRC_ALDNC.Auto.Subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
    ShooterSubsystem shooterSubsystem;
    public enum shooterState{Bank, Mid, Far, Wait}
    shooterState state;
    public ShooterCommand(ShooterSubsystem shooterSubsystem, shooterState state){
        this.shooterSubsystem = shooterSubsystem;
        this.state = state;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.configureShooter(state);
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.getBallCount() >= 3;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.configureShooter(shooterState.Wait);
    }
}
