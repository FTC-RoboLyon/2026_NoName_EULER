package org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel;

import static org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel.Drivetrain.KP_X;
import static org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel.Feeder.FeederActivePos;
import static org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel.Feeder.FeederIdlePos;
import static org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel.alianceShifter.ALLIANCE_KEY;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp
public class teleop_bot_azzie extends OpMode {
    Drivetrain drivetrain;
    Intake intake;
    Shooter shooter;
    Feeder feeder;
    Camera camera;
    private Object alliance;

    VoltageSensor voltageSensor;

    private boolean searchingForBalls = false;


    @Override
    public void init() {
         drivetrain = new Drivetrain(hardwareMap);
         intake = new Intake(hardwareMap);
         shooter = new Shooter(hardwareMap);
         feeder = new Feeder(hardwareMap);
         camera = new Camera(hardwareMap);

         voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

         alliance = blackboard.get(ALLIANCE_KEY);
         alliance = (String) alliance;
    }

    @Override
    public void loop() {
        double voltage = voltageSensor.getVoltage();

        if (gamepad1.rightBumperWasPressed()){
            searchingForBalls = !searchingForBalls;
        }
        camera.setColorProcessorEnabled(searchingForBalls);
        camera.setAprilTagProcessorEnabled(!searchingForBalls);

        if (gamepad1.left_stick_button && camera.getBearing(24) != 7 && !searchingForBalls) {
            drivetrain.AlignWithTarget(
                    camera.getBearing(alliance == "red" ? 24 : 21),
                    gamepad1.right_stick_y,
                    gamepad1.left_stick_x
            );
        }
        else if (searchingForBalls && camera.getBigestBlob() != null) {
            drivetrain.Drive(
                    gamepad1.left_stick_x,
                    1 * (1 / camera.getBigestBlob().getBoxFit().size.area()),
                    camera.getBigestBlob().getBoxFit().center.x * KP_X
            );
        }
        else {
            drivetrain.Drive(gamepad1.left_stick_x, gamepad1.right_stick_y, gamepad1.right_stick_x);
        }
        intake.setIntake_power((double) (gamepad1.right_trigger - gamepad1.left_trigger));
        feeder.setFeederPos(gamepad2.x ? FeederActivePos : FeederIdlePos);



        if (gamepad2.a)
        {
            shooter.SetShooterTargets(Shooter.nearSpeed, Shooter.nearPosHood, voltage);
        }
        else if (gamepad2.b)
        {
            shooter.SetShooterTargets(Shooter.midSpeed, Shooter.midPosHood, voltage);
        }
        else if (gamepad2.y)
        {
            shooter.SetShooterTargets(Shooter.farSpeed, Shooter.farPosHood, voltage);
        }
        else if (gamepad2.left_bumper && camera.getDistanceMeters(24)>0)
        {
            shooter.SetShooterDistanceTarget(camera.getDistanceMeters(24), voltage);
        }
        else
        {
            shooter.stopShooter();
        }

    }

    @Override
    public void stop()
    {
        camera.close();
    }

    private double leftXFilter (double input){
        double output = input;
        return  output;
    }
}
