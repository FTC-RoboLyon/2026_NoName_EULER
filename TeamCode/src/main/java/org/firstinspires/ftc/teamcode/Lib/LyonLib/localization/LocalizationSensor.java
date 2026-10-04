package org.firstinspires.ftc.teamcode.Lib.LyonLib.localization;

import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;

/**
 * Sensor contract for the fusion.
 */
public interface LocalizationSensor
{
    /**
     * Run the sensor specific initialization and save the starting position of the robot.
     * @param startingPos the starting position of the robot
     */
    default void Init(Pose2d startingPos) {}

    /**
     * Replace the estimated position by the position specified in parameters.
     * Only use when you are sure of the position you give because it will override all previous inputs of the sensor.
     * @param pos the actual robot pos
     */
    default void SetPosition(Pose2d pos) {}

    /**
     * Update sensor value and estimate the new robot position. Use only once per loop.
     * @return the estimated robot position
     */
    default Pose2d Update() {return new Pose2d();}

    /**
     * Return the position estimated by the last Update() run without updating it with the new sensors values.
     * @return the position estimated by the last Update() run
     */
    default Pose2d GetLastEstimatedPos() {return new Pose2d();}

    /**
     * Return if the sensor is ready to run normally.
     * For sensors which is impossible to know it (encoder, etc.) always return true.
     * @return if the sensor is ready to run normally.
     */
    default boolean IsReady() {return true;}

    /**
     * Return a Pose2d only containing 0 or 1 representing the axes that can be analysed by the sensor.
     * A 0 value means that the corresponding axis is not analysed by the sensor and a 1 value means
     * that this sensor can analyse the corresponding axis.
     * For example the returned value of an odometry system will be (1,1,1)
     * but for a gyro it will be (0,0,1)
     * @return the axes that can be analysed by the sensor
     */
    default Pose2d GetAnalysedAxes() {return new Pose2d();}

    /**
     * Return a Pose2d containing the standard deviation per loop for each axis.
     * If the sensor doesn't measure one of the 3 axes, a very high value must be used (like 1e10)
     * For example for a gyro the Standard Deviation must be (1e10, 1e10, hStdDev)
     * @return the standard deviation per loop for each axis
     */
    default Pose2d GetStdDev() {return new Pose2d();}

    /**
     * Return the name given to the sensor in the hardware map
     * @return the name given to the sensor in the hardware map
     */
    default String GetSensorName() {return "WhyDoIDon'tHavAName";};
}