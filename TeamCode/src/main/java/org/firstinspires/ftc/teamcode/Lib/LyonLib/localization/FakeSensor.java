package org.firstinspires.ftc.teamcode.Lib.LyonLib.localization;


import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;

public class FakeSensor implements LocalizationSensor {
    private Pose2d EstimatedPos;

    private Pose2d StdDev;
    FakeSensor(Pose2d stdDev)
    {
        this.StdDev = stdDev;
    };

    @Override
    public void SetPosition(Pose2d pos)
    {
        EstimatedPos = pos;
    }

    @Override
    public Pose2d GetLastEstimatedPos()
    {
        return EstimatedPos;
    }

    @Override
    public Pose2d GetAnalysedAxes()
    {
        return new Pose2d(1,1,1);
    }

    @Override
    public Pose2d GetStdDev()
    {
        return StdDev;
    }
}
