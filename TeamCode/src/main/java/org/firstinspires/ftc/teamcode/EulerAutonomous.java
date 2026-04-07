package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.euler.Robot;
import org.firstinspires.ftc.teamcode.euler.Step;
import org.firstinspires.ftc.teamcode.euler.steps.ForwardByTime;
import org.firstinspires.ftc.teamcode.euler.steps.Rotate;
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
        currentStepIndex = 0;
        steps = List.of(
                new ForwardByTime(1000, true),
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
        if (currentStepIndex > steps.size()) {
            // plus rien à faire
            return;
        }

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
}
