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
        boolean boost_button = gamepad1.a;
        if boost_button == true;
            double right_joystick = rightjoystick*2;

        telemetry.addData("Speed value", right_joystick);
    }


}
