package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

public class TelopCommandBase extends OpMode {
    private robotContainer robot;

    @Override
    public void init() {
        robot = new robotContainer(hardwareMap, telemetry, robotContainer.RobotMode.TELEOP);
        robot.bindCommands(gamepad1, gamepad2);

    }

    @Override
    public void loop() {

        CommandScheduler.getInstance().run();

        robot.getTelemetry().update();
        //mmmh et dans ta loop tu fais r, donc vrm ton robot ne fait r, j'avoues j'ai pas la vision la         ouais 30 seconde
    }

    @Override
    public void stop(){

    }
}
