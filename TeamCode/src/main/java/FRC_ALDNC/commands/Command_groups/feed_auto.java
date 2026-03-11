package FRC_ALDNC.commands.Command_groups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import FRC_ALDNC.SubSystem.Feeder_subsystem;
import FRC_ALDNC.SubSystem.Intake_subsystem;
import FRC_ALDNC.SubSystem.Shooter_Subsystem;
import FRC_ALDNC.commands.Collect_command;
import FRC_ALDNC.commands.Let_a_ball_pass;
import FRC_ALDNC.commands.Shoot_a_ball_command;

public class feed_auto extends SequentialCommandGroup {
    public feed_auto(Feeder_subsystem feeder, Intake_subsystem intake_subsystem, Shooter_Subsystem shooter, Telemetry telemetry){
        addCommands(
                new Collect_command(intake_subsystem, Intake_subsystem.WantedState.COLLECT, telemetry),
                new Shoot_a_ball_command(feeder, shooter, telemetry),
                new Let_a_ball_pass(feeder, telemetry),

                new Shoot_a_ball_command(feeder, shooter, telemetry),
                new Let_a_ball_pass(feeder, telemetry),

                new Shoot_a_ball_command(feeder, shooter, telemetry),
                new Let_a_ball_pass(feeder, telemetry),
                new Collect_command(intake_subsystem, Intake_subsystem.WantedState.STAND_BY, telemetry)

        );
    }
}
