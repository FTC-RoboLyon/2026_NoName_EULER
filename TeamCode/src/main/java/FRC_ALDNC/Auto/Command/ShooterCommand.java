package FRC_ALDNC.Auto.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import FRC_ALDNC.Auto.Subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
    ShooterSubsystem shooterSubsystem;
    public enum shooterState{Bank, Mid, Far, Wait}
    shooterState state;
    ElapsedTime time;
    public ShooterCommand(ShooterSubsystem shooterSubsystem, shooterState state){
        this.shooterSubsystem = shooterSubsystem;
        this.state = state;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.configureShooter(state);
        time = new ElapsedTime();
        time.reset();
    }

    @Override
    public boolean isFinished() {
        if(time.milliseconds() >= 7000){return true;}
        return shooterSubsystem.getBallCount() >= 3;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.configureShooter(shooterState.Wait);
    }
}
