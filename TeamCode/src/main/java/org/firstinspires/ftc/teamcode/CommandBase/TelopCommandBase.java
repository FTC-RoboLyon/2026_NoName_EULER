package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TelopCommandBase extends OpMode {
    private robotContainer robot;


    @Override
    public void init() {
        robot = new robotContainer(hardwareMap, telemetry, robotContainer.RobotMode.TELEOP);

    }

    @Override
    public void loop() {

    }

    @Override
    public void stop(){

    }
}
