package FRC_ALDNC.Auto.Command;



import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import FRC_ALDNC.Auto.Subsystems.FeederSubsystem;
import FRC_ALDNC.Auto.Subsystems.ShooterSubsystem;


public class FeederCommand extends CommandBase {
    int v = 0;

    FeederSubsystem feeder;
    ShooterSubsystem shooterSubsystem;
    Telemetry telemetrY;
    ElapsedTime time;
    public FeederCommand (FeederSubsystem feeder, Telemetry telemetry, ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        telemetrY = telemetry;
        this.feeder = feeder;
        addRequirements( feeder);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        telemetrY.addLine("shoot a ball has execute");
        telemetrY.update();
        if(shooterSubsystem.bonneVitesseOuPas() && v == 0) {
            feeder.setfeeder_wanted_state(FeederSubsystem.Feeder_wanted_state.Haut);
            telemetrY.addLine("shoot a ball is initialize");
            telemetrY.update();
            time = new ElapsedTime();
            time.reset();
            v = 1;
        }

    }




    @Override
    public void end(boolean interrupted) {
        telemetrY.addLine("shoot a ball has end");
        telemetrY.update();
        feeder.setfeeder_wanted_state(FeederSubsystem.Feeder_wanted_state.bas);

    }

    @Override
    public boolean isFinished() {
        //return Utils.IsInRange(feeder.get_Pos(), posFeed, 0.005);
        return v == 1 && time.milliseconds() >= 250;

    }
}
