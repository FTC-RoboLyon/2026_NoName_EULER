package FRC_ALDNC.commands.Command_groups;

import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.posviseur_mid;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import FRC_ALDNC.SubSystem.Camera_subsystem;
import FRC_ALDNC.SubSystem.Drive_Train;
import FRC_ALDNC.SubSystem.Feeder_subsystem;
import FRC_ALDNC.SubSystem.Intake_subsystem;
import FRC_ALDNC.SubSystem.Shooter_Subsystem;
import FRC_ALDNC.commands.AlignToTarget;
import FRC_ALDNC.commands.Collect_command;
import FRC_ALDNC.commands.Configure_shooter;
import FRC_ALDNC.commands.Stop_shooter;

public class Shoot_auto extends SequentialCommandGroup {
    public Shoot_auto (Shooter_Subsystem shooter_subsystem,
                       Feeder_subsystem feederSubsystem,
                       Shooter_Subsystem.WantedState postir,
                       Intake_subsystem intake, Drive_Train chassis,
                       Camera_subsystem cam ,Telemetry telemetry){


        addCommands(
                new InstantCommand(()-> shooter_subsystem.viseur.setPosition(posviseur_mid)),
                new Configure_shooter(shooter_subsystem, cam, postir, false, telemetry),
                new WaitCommand(300),
                //new AlignToTarget(chassis, cam, shooter_subsystem, false),
                new feed_auto(feederSubsystem, intake, shooter_subsystem, telemetry),
                new Stop_shooter(shooter_subsystem, telemetry)

                );
    }
}
