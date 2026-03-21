package FRC_ALDNC.Auto.Command;



import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import FRC_ALDNC.Auto.Subsystems.FeederSubsystem;
import FRC_ALDNC.Auto.Subsystems.ShooterSubsystem;


public class FeederCommand extends CommandBase {
    int v = 0;
    enum etat {haut, bas}
    etat state;

    FeederSubsystem feeder;
    ShooterSubsystem shooterSubsystem;
    Telemetry telemetrY;
    ElapsedTime time;
    public FeederCommand (FeederSubsystem feeder, Telemetry telemetry, ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        telemetrY = telemetry;
        this.feeder = feeder;
        state = etat.haut;
        addRequirements( feeder);
    }

    @Override
    public void execute() {
        if(shooterSubsystem.bonneVitesseOuPas()){
            switch (state){
                case haut:
                    feeder.setfeeder_wanted_state(FeederSubsystem.Feeder_wanted_state.Haut);
                    time = new ElapsedTime();
                    time.reset();
                    state = etat.bas;
                    v = 0;
                    break;
                case bas:
                    if(time.milliseconds() >= 250 && v == 0) {
                        feeder.setfeeder_wanted_state(FeederSubsystem.Feeder_wanted_state.bas);
                        time.reset();
                        v = 1;
                    }
                    if (time.milliseconds() >= 3000) state = etat.haut;
                    break;
            }
        }

    }

    @Override
    public void end(boolean interrupted) {
        feeder.setfeeder_wanted_state(FeederSubsystem.Feeder_wanted_state.bas);
    }
}
