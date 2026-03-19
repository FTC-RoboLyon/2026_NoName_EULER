package FRC_ALDNC.programmes;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.BooleanSupplier;

import FRC_ALDNC.ALDNC_container;
import FRC_ALDNC.SubSystem.Shooter_Subsystem;
import FRC_ALDNC.commands.AlignToTarget;
//import FRC_ALDNC.commands.Command_groups.Shoot_auto;
import FRC_ALDNC.commands.Drive_command;
import FRC_ALDNC.commands.Wait_for_start;
@Autonomous(name = "FRC ALDNC auto RED Far", group = "FRC_style")
public class auto_red_far extends CommandOpMode {
    ALDNC_container robot;
    @Override
    public void initialize() {
        robot = new ALDNC_container(hardwareMap, ALDNC_container.RobotMode.AUTO_RED, new GamepadEx(gamepad1), telemetry);
        BooleanSupplier Wait_for_start = this::opModeIsActive;

        schedule(new SequentialCommandGroup(
                new Wait_for_start(Wait_for_start),
                //new AlignToTarget(robot.Chassis(), robot.Camera(), robot.Shooter(),false ),
                //new Shoot_auto(robot.Shooter(), robot.Feeder(), Shooter_Subsystem.WantedState.SHOOT_FAR, robot.Intake(), robot.Chassis(), robot.Camera(), telemetry),//tirer les trois première

        new Drive_command(robot.Chassis(), 1, 1, 300, telemetry)//Aller a la pos tir mid
                //new WaitCommand(100),
                // new Drive_command(robot.Chassis(), 0, -0.4, 550, telemetry),//se tourner vers le menisque
        ));

}
}
