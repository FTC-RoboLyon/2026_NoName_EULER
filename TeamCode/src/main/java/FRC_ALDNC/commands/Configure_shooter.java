package FRC_ALDNC.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import FRC_ALDNC.SubSystem.Camera_subsystem;
import FRC_ALDNC.SubSystem.Shooter_Subsystem;

public class Configure_shooter extends CommandBase {
    Shooter_Subsystem shooter;

    Shooter_Subsystem.WantedState postir;
    Camera_subsystem came;
    boolean toggle;
    boolean isConfigured;
    Telemetry telemetrY;
    public Configure_shooter(Shooter_Subsystem shooter, Camera_subsystem cam, Shooter_Subsystem.WantedState postir, boolean toggle, Telemetry telemetry){
        this.toggle = toggle;
        came = cam;
        telemetrY = telemetry;
        this.shooter = shooter;
        this.postir = postir;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        if (postir == Shooter_Subsystem.WantedState.AUTO ) {
            if (came.getActual_detection() != null) {
                shooter.calculate_postir(came.getActual_detection().ftcPose.y);
                isConfigured = true;
                shooter.setShooter_state(postir);
            }
        } else{
            isConfigured = true;
            shooter.setShooter_state(postir);
        }
        telemetrY.addLine("Configure shooter is initialize");
        telemetrY.update();
    }

    @Override
    public void execute() {
        if (isConfigured)
            return;
        else {
            if (came.getActual_detection() != null) {
                shooter.calculate_postir(came.getActual_detection().ftcPose.y);
                isConfigured = true;
                shooter.setShooter_state(postir);
            }
        }
        telemetrY.addLine("Configure shooter is execute");
        telemetrY.update();

    }

    @Override
    public void end(boolean interrupted) {
        if (toggle)
            shooter.setShooter_state(Shooter_Subsystem.WantedState.WAIT);
        telemetrY.addLine("Configure shooter has end");
        telemetrY.update();
    }

    @Override
    public boolean isFinished() {
        if (!toggle)
            return shooter.getShooterSysState() == Shooter_Subsystem.SystemState.READY_TO_SHOOT;
        return false;
    }
}
