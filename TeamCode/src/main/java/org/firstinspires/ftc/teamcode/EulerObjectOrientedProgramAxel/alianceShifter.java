package org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.security.acl.Group;

@TeleOp(name = "alianceShifter")
public class alianceShifter extends OpMode {
    public static final String ALLIANCE_KEY = "Alliance";
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        if (gamepad1.leftBumperWasPressed()) {
            blackboard.put(ALLIANCE_KEY, "RED");
        } else if (gamepad1.rightBumperWasPressed()) {
            blackboard.put(ALLIANCE_KEY, "BLUE");
        }
        telemetry.addData("Alliance", blackboard.get(ALLIANCE_KEY));
    }
}
