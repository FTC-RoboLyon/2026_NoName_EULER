package org.firstinspires.ftc.teamcode.Euler_niveau_supérieur_axel;

import static org.firstinspires.ftc.teamcode.Euler_niveau_supérieur_axel.Feeder.PosFeederActive;
import static org.firstinspires.ftc.teamcode.Euler_niveau_supérieur_axel.Feeder.PosFeederIddle;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class teleop_bot_azzie extends OpMode {
    DriveTrain base;
    Intake intake;
    Shooter shooter;
    Feeder feeder;
    Path path;
    VoltageSensor voltageSensor;

    @Override
    public void init() {
         base = new DriveTrain(hardwareMap);
         intake = new Intake(hardwareMap);
         shooter = new Shooter(hardwareMap);
         feeder = new Feeder(hardwareMap);
         path = new Path(hardwareMap);

         voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }

    @Override
    public void loop() {
        double voltage = voltageSensor.getVoltage();

        base.Drive(gamepad1.left_stick_x, gamepad1.right_stick_y);
        intake.setIntake_power((double) (gamepad1.right_trigger - gamepad1.left_trigger));
        feeder.setPosFeeder( gamepad2.x ? PosFeederActive : PosFeederIddle);
        path.setPathPower(Math.abs(intake.intake_power) > 0 ? 1 : -1);        //inverser les 1 et -1 en fonction du sens voulu

        if (gamepad2.a)
            shooter.setShooterState(Shooter.ShooterState.VeloShootFar);
        else if (gamepad2.b) {
            shooter.setShooterState(Shooter.ShooterState.VeloShootNear);
        } else if (gamepad2.y) {
            shooter.setShooterState(Shooter.ShooterState.VeloShootMid);
        }


        base.loop(voltage);
        intake.intake_loop();
        shooter.shooterLoop(voltage);
        feeder.feeder_loop();
        path.Path_loop();
    }
}
