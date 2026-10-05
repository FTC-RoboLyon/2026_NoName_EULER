package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TelopCommandBase extends OpMode {
    private robotContainer robot;


    @Override
    public void init() {
        robot = new robotContainer(hardwareMap, telemetry, gamepad1, gamepad2, robotContainer.Periode.TELEOP);

    }

    @Override
    public void loop() {
        //mmmh et dans ta loop tu fais r, donc vrm ton robot ne fait r, j'avoues j'ai pas la vision la
    }

    @Override
    public void stop(){

    }
}
