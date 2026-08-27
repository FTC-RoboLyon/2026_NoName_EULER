package org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel;

import static org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel.Feeder.FeederActivePos;
import static org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel.Feeder.FeederIdlePos;
import static org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel.allianceShifter.ALLIANCE_KEY;

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
    private String alliance;

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

         alliance = (String) blackboard.get(ALLIANCE_KEY);
    }

    @Override
    public void loop() {
        double voltage = voltageSensor.getVoltage();

        if (gamepad1.left_stick_button && camera.getBearing(alliance == "red" ? 24 : 21) != 7) {
            //et d'ailleurs pourquoi ne pas profiter de ton gyro et de ta localisation pour un premier alignement tant qu'il ne voit pas l'april tag -> pas encore fait
            drivetrain.headToTarget(
                    camera.getBearing(alliance == "red" ? 24 : 21),
                    gamepad1.right_stick_y,
                    -gamepad1.left_stick_x
            );
        }

        else {
            drivetrain.Drive(-gamepad1.left_stick_x, gamepad1.right_stick_y, -gamepad1.right_stick_x, true);
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
        else if (gamepad2.left_bumper && camera.getDistanceMeters(alliance == "red" ? 24 : 21)>0)
        {
            shooter.SetShooterDistanceTarget(camera.getDistanceMeters(alliance == "red" ? 24 : 21), voltage);
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

    private double leftXFilter (double input){//mmmmmmmmhh... Je pense que c'est la fonction la plus clean que tu ait jamais faite vrm bravo !!! ;)
        double output = input;
        return  output;
    }
}
