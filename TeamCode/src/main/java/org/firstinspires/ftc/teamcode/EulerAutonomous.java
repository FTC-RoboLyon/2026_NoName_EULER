package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.euler.Robot;
import org.firstinspires.ftc.teamcode.euler.Step;
import org.firstinspires.ftc.teamcode.euler.steps.ForwardByTime;
import org.firstinspires.ftc.teamcode.euler.steps.GoToCoordinate;
import org.firstinspires.ftc.teamcode.euler.steps.Rotate;
import org.firstinspires.ftc.teamcode.euler.steps.Shoot;
import org.firstinspires.ftc.teamcode.euler.steps.StartCollect;
import org.firstinspires.ftc.teamcode.euler.steps.StopCollect;

import java.util.List;

@Autonomous(preselectTeleOp = "EulerTeleop", group = "Euler")
public class EulerAutonomous extends OpMode {
    private Robot robot;

    private List<Step> steps;
    private int currentStepIndex;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        // position de depart shoot rouge middle
        robot.getOdometry().reset(new Pose2D(DistanceUnit.MM, 30, 30, AngleUnit.DEGREES, 45));

        currentStepIndex = 0;
        steps = List.of(
                new Shoot(Shoot.ShootPosition.MIDDLE, 3),
                new GoToCoordinate(new Pose2D(DistanceUnit.MM, -15, 15, AngleUnit.DEGREES, -180)),
                new Rotate(90),
                new StartCollect(),
                new ForwardByTime(2000, true),
                new StopCollect(),
                new ForwardByTime(2000, false),
                new Rotate(90)
        );
    }

    @Override
    public void loop() {
        if (currentStepIndex >= steps.size()) {
            // plus rien à faire
            return;
        }

        displayTelemetry();

        Step currentStep = steps.get(currentStepIndex);

        if (!currentStep.isInitialized()) {
            currentStep.init(robot);
        }

        currentStep.run(robot);

        if (currentStep.isFinished()) {
            currentStep.finish(robot);
            currentStepIndex++; // passe à la step suivante
        }
    }

    private void displayTelemetry() {
        telemetry.addData("Step", steps.get(currentStepIndex).getClass().getName());
        telemetry.addData("Step index", currentStepIndex + "/" + steps.size());

        robot.getTelemetries()
                .forEach(robotTelemetry -> {
                    telemetry.addData(robotTelemetry.getCaption(), robotTelemetry.getValue());
                });

        telemetry.update();
    }
}
