package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

public class TelopCommandBase extends OpMode {
    private robotContainer robot;

    @Override
    public void init() {
        robot = new robotContainer(hardwareMap, telemetry, robotContainer.RobotMode.TELEOP);
        robot.bindCommands(gamepad1, gamepad2);

        telemetry.addData("Alliance", robot.getAlliance());
    }

    @Override
    public void loop() {

        CommandScheduler.getInstance().run();

        telemetry.update();
    }

    @Override
    public void stop(){

    }
}
