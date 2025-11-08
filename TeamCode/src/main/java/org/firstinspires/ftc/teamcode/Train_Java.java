package org.firstinspires.ftc.teamcode;

@TeleOp()
public class Train_Java extends OpMode {
    @Override
    public void init() {
        String nom = "Rafael";
        telemetry.addData("EulerTeam");
        telemetry.addData(nom);
    }

    public void loop() {
        double right_joystick = -gamepad1.right_stick_y;
        telemetry.addData("Speed value", right_joystick);
    }


}
