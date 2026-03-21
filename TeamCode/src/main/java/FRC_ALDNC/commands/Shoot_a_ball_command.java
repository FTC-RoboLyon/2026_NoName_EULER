package FRC_ALDNC.commands;


import static FRC_ALDNC.CONSTANT.constante_feeder.posFeed;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.robocol.Command;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import FRC_ALDNC.SubSystem.Feeder_subsystem;
import FRC_ALDNC.SubSystem.Shooter_Subsystem;
import lib.Utils;

public class Shoot_a_ball_command extends CommandBase {

    Feeder_subsystem feeder;
    Shooter_Subsystem shooter;
    ElapsedTime time;

    private RevBlinkinLedDriver led;
    public Shoot_a_ball_command (Feeder_subsystem feeder, Shooter_Subsystem shooter, RevBlinkinLedDriver led){

        this.shooter = shooter;
        this.feeder = feeder;
        this.led = led;
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        feeder.setfeeder_wanted_state(Feeder_subsystem.Feeder_wanted_state.Haut);

        time = new ElapsedTime();
        time.reset();
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
    }

    @Override
    public void end (boolean interrupted)
    {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }


    @Override
    public boolean isFinished() {
        //return Utils.IsInRange(feeder.get_Pos(), posFeed, 0.005);
        return time.milliseconds() >= 250;
    }
}
