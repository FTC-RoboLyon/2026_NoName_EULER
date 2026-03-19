package FRC_ALDNC.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import FRC_ALDNC.SubSystem.Camera_subsystem;
import FRC_ALDNC.SubSystem.Shooter_Subsystem;

public class Configure_shooter extends CommandBase {
    Shooter_Subsystem shooter;

    Shooter_Subsystem.WantedState postir;
    boolean toggle;

    public Configure_shooter(Shooter_Subsystem shooter, Shooter_Subsystem.WantedState postir, boolean toggle){
        this.toggle = toggle;
        this.shooter = shooter;
        this.postir = postir;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooter_state(postir);
    }


    @Override
    public void end(boolean interrupted) {
        if (toggle)
            shooter.setShooter_state(Shooter_Subsystem.WantedState.WAIT);

    }

    @Override
    public boolean isFinished() {
        if (!toggle)
            return shooter.getShooterSysState() == Shooter_Subsystem.SystemState.READY_TO_SHOOT;
        return false;
    }
}
