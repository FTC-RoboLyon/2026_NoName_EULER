package FRC_ALDNC.programmes;

import static FRC_ALDNC.CONSTANT.constante_joystick_and_base.forward_auto;
import static FRC_ALDNC.CONSTANT.constante_joystick_and_base.time_auto;
import static FRC_ALDNC.CONSTANT.constante_joystick_and_base.turn_auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Map;
import java.util.function.BooleanSupplier;

import FRC_ALDNC.ALDNC_container;
import FRC_ALDNC.SubSystem.Intake_subsystem;
import FRC_ALDNC.SubSystem.Shooter_Subsystem;
import FRC_ALDNC.commands.AlignToTarget;
import FRC_ALDNC.commands.Collect_command;
import FRC_ALDNC.commands.Drive_command;
import FRC_ALDNC.commands.Command_groups.Shoot_auto;
import FRC_ALDNC.commands.Finf_order_artefact;
import FRC_ALDNC.commands.Wait_for_start;

@Config
@Autonomous(name = "FRC ALDNC auto RED", group = "FRC_style")
public class FRC_ALDNC_auto_red extends CommandOpMode {
    public  static double forward_auto = -1;
    public  static double turn_auto = 0;
    public  static double time_auto = 1000;
    ALDNC_container robot;
    @Override
    public void initialize() {
        robot = new ALDNC_container(hardwareMap, ALDNC_container.RobotMode.AUTO_RED, new GamepadEx(gamepad1), telemetry);
        BooleanSupplier Wait_for_start = this::opModeIsActive;

            schedule(new SequentialCommandGroup(
                            new Wait_for_start(Wait_for_start),
                            new Drive_command(robot.Chassis(), -1, -0.05
                                    , 1050 ), //Aller a la pos tir mid
                            //new WaitCommand(100),
                            new Shoot_auto(robot.Shooter(), robot.Feeder(), Shooter_Subsystem.WantedState.SHOOT_MID, robot.Intake(), robot.Chassis(), robot.Camera()),//tirer les trois première
                           // new Drive_command(robot.Chassis(), 0, -0.4, 550),//se tourner vers le menisque
                            new WaitCommand(100),
                    new Drive_command(robot.Chassis(), 0, 0.4, 500), //a peu près 400, a revoir
                    new Collect_command(robot.Intake(), Intake_subsystem.WantedState.COLLECT), // activer l'intake
                    new Drive_command(robot.Chassis(), 0.9, 0.1, 300),// avancer pr récupérer les balle
                    new WaitCommand(100),
                    new Drive_command(robot.Chassis(), 0.225, 0.001, 3100),// s'arrêter pdnt un tt petit moment
                    new WaitCommand(100),
                            new Collect_command(robot.Intake(), Intake_subsystem.WantedState.COLLECT)
                    //new Drive_command(robot.Chassis(), -1, 0, 1250), // reculer//                       les valeurs ici sont égale a la deuxième ligne au dessus
                    //new WaitCommand(100),
                    //new Drive_command(robot.Chassis(), 0, -0.5, 575),
                    //new Shoot_auto(robot.Shooter(), robot.Feeder(), Shooter_Subsystem.WantedState.SHOOT_MID, robot.Intake(), robot.Chassis(), robot.Camera()),//tirer les trois première
                    //new Drive_command(robot.Chassis(),1, 1, 500)


        //new Drive_command(robot.Chassis(), 0, 0, 100), //attendre un peu
                            //new Collect_command(robot.Intake(), Intake_subsystem.WantedState.STAND_BY), // arreter l'intake
                            //new Drive_command(robot.Chassis(), 1, 1, 500), //se tourner                            les valeurs ici sont égale a la troisieme ligne de ce sequential group
                            //new Drive_command(robot.Chassis(), 0, 0, 100), //attendre un peu
                            //new Drive_command(robot.Chassis(), 1, 1, 500),// avancer a la launch zone            les valeurs ici sont égale a la premiere ligne de ce sequential group
                            //new Drive_command(robot.Chassis(), 0, 0, 100), //attendre un peu
                            //new Drive_command(robot.Chassis(), 1, 1, 500), //se tourner
                            //new Shoot_auto(robot.Shooter(), robot.Feeder(), Shooter_Subsystem.WantedState.SHOOT_MID, robot.Intake(), robot.Chassis(), robot.Camera()) //tirer les trois deuxième//

                            //new Finf_order_artefact(robot.Camera(), robot), // lire l'ordre

                            //new SelectCommand(
                            //        Map.of(
                            //                ALDNC_container.Artefact_order.GPP, new SequentialCommandGroup(                  // <---- le plus loin de la grande far zone
                            //                        new Drive_command(robot.Chassis(), 1, 1, 500), // reculer
                            //                        new Drive_command(robot.Chassis(), 0, 0, 100), //attendre un peu
                            //                        new Drive_command(robot.Chassis(), 1, 1, 500), //se tourner
                            //                        new Drive_command(robot.Chassis(), 0, 0, 100), //attendre un peu
                            //                        new Collect_command(robot.Intake(), Intake_subsystem.WantedState.COLLECT), // activer l'intake
                            //                        new Drive_command(robot.Chassis(), 1, 1, 500),// avancer pr récupérer les balle
                            //                        new Drive_command(robot.Chassis(), 0, 0, 100),// s'arrêter pdnt un tt petit moment
                            //                        new Drive_command(robot.Chassis(), 1, 1, 500), // reculer                          les valeurs ici sont égale a la deuxième ligne au dessus
                            //                        new Drive_command(robot.Chassis(), 0, 0, 100), //attendre un peu
                            //                        new Collect_command(robot.Intake(), Intake_subsystem.WantedState.STAND_BY), // arreter l'intake
                            //                        new Drive_command(robot.Chassis(), 1, 1, 500), //se tourner                            les valeurs ici sont égale a la troisieme ligne de ce sequential group
                            //                        new Drive_command(robot.Chassis(), 0, 0, 100), //attendre un peu
                            //                        new Drive_command(robot.Chassis(), 1, 1, 500),// avancer a la launch zone            les valeurs ici sont égale a la premiere ligne de ce sequential group
                            //                        new Drive_command(robot.Chassis(), 0, 0, 100), //attendre un peu
                            //                        new Drive_command(robot.Chassis(), 1, 1, 500), //se tourner
                            //                        new Shoot_auto(robot.Shooter(), robot.Feeder(), Shooter_Subsystem.WantedState.SHOOT_MID, robot.Intake(), robot.Chassis(), robot.Camera()) //tirer les trois deuxième
                            //                ),
                            //                ALDNC_container.Artefact_order.PGP, new SequentialCommandGroup(
                            //                        new Drive_command(robot.Chassis(), 1, 1, 500), // reculer
                            //                        new Drive_command(robot.Chassis(), 0, 0, 100), //attendre un peu
                            //                        new Drive_command(robot.Chassis(), 1, 1, 500), //se tourner
                            //                        new Drive_command(robot.Chassis(), 0, 0, 100), //attendre un peu
                            //                        new Collect_command(robot.Intake(), Intake_subsystem.WantedState.COLLECT), // activer l'intake
                            //                        new Drive_command(robot.Chassis(), 1, 1, 500),// avancer pr récupérer les balle
                            //                        new Drive_command(robot.Chassis(), 0, 0, 100),// s'arrêter pdnt un tt petit moment
                            //                        new Drive_command(robot.Chassis(), 1, 1, 500), // reculer                           les valeurs ici sont égale a la deuxième ligne au dessus
                            //                        new Drive_command(robot.Chassis(), 0, 0, 100), //attendre un peu
                            //                        new Collect_command(robot.Intake(), Intake_subsystem.WantedState.STAND_BY), // arreter l'intake
                            //                        new Drive_command(robot.Chassis(), 1, 1, 500), //se tourner                          les valeurs ici sont égale a la troisieme ligne de ce sequential group
                            //                        new Drive_command(robot.Chassis(), 0, 0, 100), //attendre un peu
                            //                        new Drive_command(robot.Chassis(), 1, 1, 500),// avancer a la launch zone           les valeurs ici sont égale a la premiere ligne de ce sequential group
                            //                        new Drive_command(robot.Chassis(), 0, 0, 100), //attendre un peu
                            //                        new Drive_command(robot.Chassis(), 1, 1, 500), //se tourner
                            //                        new Shoot_auto(robot.Shooter(), robot.Feeder(), Shooter_Subsystem.WantedState.SHOOT_MID, robot.Intake(), robot.Chassis(), robot.Camera()) //tirer les trois deuxième
                            //                ),
                            //                ALDNC_container.Artefact_order.PPG, new SequentialCommandGroup(
                            //                        new Drive_command(robot.Chassis(), 1, 1, 500), // reculer
                            //                        new Drive_command(robot.Chassis(), 0, 0, 100), //attendre un peu
                            //                        new Drive_command(robot.Chassis(), 1, 1, 500), //se tourner
                            //                        new Drive_command(robot.Chassis(), 0, 0, 100), //attendre un peu
                            //                        new Collect_command(robot.Intake(), Intake_subsystem.WantedState.COLLECT), // activer l'intake
                            //                        new Drive_command(robot.Chassis(), 1, 1, 500),// avancer pr récupérer les balle
                            //                        new Drive_command(robot.Chassis(), 0, 0, 100),// s'arrêter pdnt un tt petit moment
                            //                        new Drive_command(robot.Chassis(), 1, 1, 500), // reculer                                  les valeurs ici sont égale a la deuxième ligne au dessus
                            //                        new Drive_command(robot.Chassis(), 0, 0, 100), //attendre un peu
                            //                        new Collect_command(robot.Intake(), Intake_subsystem.WantedState.STAND_BY), // arreter l'intake
                            //                        new Drive_command(robot.Chassis(), 1, 1, 500), //se tourner                              les valeurs ici sont égale a la troisieme ligne de ce sequential group
                            //                        new Drive_command(robot.Chassis(), 0, 0, 100), //attendre un peu
                            //                        new Drive_command(robot.Chassis(), 1, 1, 500),// avancer a la launch zone               les valeurs ici sont égale a la premiere ligne de ce sequential group
                            //                        new Drive_command(robot.Chassis(), 0, 0, 100), //attendre un peu
                            //                        new Drive_command(robot.Chassis(), 1, 1, 500), //se tourner
                            //                        new Shoot_auto(robot.Shooter(), robot.Feeder(), Shooter_Subsystem.WantedState.SHOOT_MID, robot.Intake(), robot.Chassis(), robot.Camera()) //tirer les trois deuxième
                            //                )
                            //        ), robot :: Get_actual_artefact_order
                            //)
                    )
            );


    }
}
