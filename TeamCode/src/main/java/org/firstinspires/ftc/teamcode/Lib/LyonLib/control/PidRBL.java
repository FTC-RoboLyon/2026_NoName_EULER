/*******************************************************************************
 * 
 * File        : PidRBL.java (v5.0)
 * Library     : LyonLib - FTC edition
 * Description : Advanced PID controller class implementing 
 *               Proportional-Integral-Derivative control. Supports real-time 
 *               update, input/output clamping, and continuous inputs (e.g. angle 
 *               wrap-around).
 * 
 * Authors     : Gaspard (2023), last update by AKA (06/2026) 
 *                                and inspired by Team 1678
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/

package org.firstinspires.ftc.teamcode.Lib.LyonLib.control;

import org.firstinspires.ftc.teamcode.Lib.LyonLib.logging.DebugUtils;
import org.firstinspires.ftc.teamcode.Lib.LyonLib.utils.UtilsRBL;


public class PidRBL 
{

    // factor for Proportional gain
    private double kP;

    // factor for Integral gain
    private double kI;

    // factor for Derivative gain
    private double kD;

    // Anti-windup back-calculation gain
    private double kAW = 0.0;

    // Min output value
    private double outputMin = -1.0;

    // Max output value
    private double outputMax = 1.0;

    // Min setpoint value allowed
    private double inputMin = 0.0;

    // Max setpoint value allowed
    private double inputMax = 0.0;

    // do the endpoints wrap around?
    private boolean isContinuous = false;

    // When set to true, the inputs will be constrained within specified limits.
    private boolean isInputLimitsActive = false;

    private boolean isfirstRun = true;

    // the prior measurement for derivative calculation
    private double previousMeasurement = 0.0;

    // Total accumulated error for integral term
    private double integrative = 0.0;

    // Desired setpoint
    private double setpoint = 0.0;

    // Error between the setpoint and the measurement
    private double currentError = 0.0;

    // Output of the PID controller
    private double output = 0.0;

    // Last time the PID controller was updated
    private double lastTimestamp = 0.0;

    // Time step for the PID controller, default in FRC is 20ms
    private double dt = 0.02;

    public PidRBL() 
    {
        this(0.0, 0.0, 0.0);
    }

    public PidRBL(double kP, double kI, double kD) 
    {
        setGains(kP, kI, kD);
    }

    /**
     * @brief Sets the PID (Proportional, Integral, Derivative) gains for the controller.
     *
     * @param kP The proportional gain, which determines the reaction to the current error.
     * @param kI The integral gain, which determines the reaction based on the accumulation of past errors.
     * @param kD The derivative gain, which determines the reaction based on the rate of change of the error.
     */
    public void setGains(double kP, double kI, double kD) 
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        if (this.kI != 0.0) 
        {
            kAW = 1.0 / this.kI; // Set anti-windup gain based on integral gain
        } 
        else 
        {
            kAW = 0.0; // No anti-windup if integral gain is zero
        }
    }

    /**
     * @brief Sets the desired setpoint for the PID controller, clamping it within the allowed input range.
     *
     * @param setpoint The desired setpoint value for the PID controller.
     */
    public void setSetpoint(double setpoint) 
    {
        if (isContinuous && (inputMax - inputMin) > 0.0)
            this.setpoint = UtilsRBL.inputModulus(setpoint, inputMin, inputMax);
        else if (isInputLimitsActive)
            this.setpoint = UtilsRBL.clamp(inputMin, setpoint, inputMax);
        else
            this.setpoint = setpoint;
    }

    /**
     * Sets the output limits for the PID controller.
     *
     * @param min The minimum output value that the PID controller can produce.
     * @param max The maximum output value that the PID controller can produce.
     */
    public void setOutputLimits(double min, double max) 
    {
    
        outputMin = min;
        outputMax = max;

        if (DebugUtils.assertion((outputMin < outputMax), "PidRBL : Output minimum must be less than or equal to output maximum.")) 
        {
            // Ensure min and max are in the correct order in competition mode (where we don't throw an exception)
            outputMin = Math.min(min, max);
            outputMax = Math.max(min, max);
        }
    }   

    /**
     * @brief Sets the input limits (acceptable range for setpoint values) for the PID controller.
     *
     * @param min The minimum allowable input value.
     * @param max The maximum allowable input value.
     * 
     * @implNote This method ensures that the minimum input value is less than the maximum input value. 
     * If this condition is not met, an assertion error will be thrown.
     */
    public void setInputLimits(double min, double max) 
    {

        DebugUtils.assertion((min < max), "PidRBL : Input minimum must be less than the input maximum.");
        inputMin = Math.min(min, max);
        inputMax = Math.max(min, max);
    }

    /**
     * @brief Sets the input limits for the PID controller.
     *
     * This function activates or deactivates the input limits for the PID controller
     * based on the provided boolean parameter. When input limits are active, the
     * controller will constrain the input values within a predefined range.
     *
     * @param isActive A boolean value indicating whether input limits should be active.
     */
    public void setInputLimits(boolean isActive) 
    {
        isInputLimitsActive = isActive;
    }

    /**
     * @brief Sets whether the PID controller should handle inputs as continuous.
     *
     * When enabled, the controller will treat the input range as circular, allowing
     * for seamless transitions between the minimum and maximum values.
     *
     * @param isContinuous A boolean indicating whether circular input handling is enabled.
     */
    public void setContinuous(boolean isContinuous) 
    {
        this.isContinuous = isContinuous;
    }

    public double getKP() { return kP; }
    public double getKI() { return kI; }
    public double getKD() { return kD; }
    public double getError() { return currentError; }
    public double getSetpoint() { return setpoint; }
    /**
     * @brief Retrieves the current state of the PID controller as a formatted string.
     *
     * @return A string containing the formatted state of the PID controller.
     */
    public String getState() 
    {
        StringBuilder state = new StringBuilder("PID State: \n");

        state.append("KP: ").append(kP).append("\n");
        state.append("KI: ").append(kI).append("\n");
        state.append("KD: ").append(kD).append("\n");
        state.append("KAW: ").append(kAW).append("\n");
        state.append("Setpoint: ").append(setpoint).append("\n");
        state.append("Input Min: ").append(inputMin).append("\n");
        state.append("Input Max: ").append(inputMax).append("\n");  
        state.append("Current Error: ").append(currentError).append("\n");
        state.append("Output: ").append(output).append("\n");
        state.append("Output Min: ").append(outputMin).append("\n");
        state.append("Output Max: ").append(outputMax).append("\n");
        state.append("Last Timestamp: ").append(lastTimestamp).append("\n");
        state.append("Delta Time: ").append(dt).append("\n");
        state.append("Is Continuous: ").append(isContinuous).append("\n");
        state.append("Integrative: ").append(integrative).append("\n");
        state.append("Previous Measurement: ").append(previousMeasurement).append("\n");
        return state.toString();
  }


    /**
     * Calculates the PID output using real timestamp.
     *
     * @param measurement The current value of the system being controlled.
     * @param timestamp The current time in seconds, used for real-time calculations.
     * @return The PID output value.
     */
    public double calculateWithRealTime(double measurement, double timestamp) 
    {

        if (isfirstRun) {
            lastTimestamp = timestamp;
            previousMeasurement = measurement;
            isfirstRun = false;
        }
        else 
        {
            // Recalculate dt based on timestamps for real-time systems
            dt = timestamp - lastTimestamp;

            lastTimestamp = timestamp;

            if (dt <= 0.0) 
            {
                DebugUtils.assertion(false, "PidRBL: Timestamp must be non-decreasing. dt must be positive.");
                return output; //prevent division by zero or negative time step, return last output
            }
        }

        currentError = setpoint - measurement;

        if (isContinuous) 
        {
            double range = inputMax - inputMin;
            if (range > 0.0)
                currentError = UtilsRBL.inputModulus(currentError, -range / 2.0, range / 2.0);
            else
                DebugUtils.assertion(false, "PidRBL: Input range must be positive for continuous mode.");
        }

        integrative += currentError * dt;

        double unsaturated =
                kP * currentError +
                kI * integrative +
                (-kD * ((measurement - previousMeasurement) / dt));

        previousMeasurement = measurement;

        // Clamp output within allowed range
        if (unsaturated > outputMax)
            output = outputMax;
        else if (unsaturated < outputMin)
            output = outputMin;
        else
            output = unsaturated;

        // === ANTI-WINDUP BACK-CALCULATION
        integrative += kAW * (output - unsaturated) * dt;

        return output;
    }

    public double calculateWithRealTime(double setpoint, double measurement, double timestamp) 
    {
        setSetpoint(setpoint);
        return calculateWithRealTime(measurement, timestamp);
    }

    /**
     * Calculates the PID output using theoretical delta time (0.02s in FRC).
     */
    public double calculate(double measurement) 
    {
        return calculateWithRealTime(measurement, lastTimestamp + 0.02);
    }

    public double calculate(double setpoint, double measurement) 
    {
        setSetpoint(setpoint);
        return calculateWithRealTime(measurement, lastTimestamp + 0.02);
    }

    /**
     * @brief Resets the PID controller state.
     * 
     * @param measurement The current measurement value to reset the controller with.
     */
    public void reset(double measurement) 
    {

        setSetpoint(measurement); // avoid derivative kick
        previousMeasurement = measurement;
        currentError = 0.0;
        output = 0.0;

        isfirstRun = true; // Reset first run flag to ensure proper timestamp handling on next calculation

        resetIntegrative();
    }

    /**
     * @brief Resets the PID controller state and updates the last timestamp.
     * 
     * @param measurement The current measurement value to reset the controller with.
     * @param timestamp The current time in seconds, used to update the last timestamp for real-time calculations.
     */
    public void reset(double measurement, double timestamp) 
    {

        lastTimestamp = timestamp;
        reset(measurement);
    }

    /**
     * @brief Resets the integrative term accumulator to zero.
     */
    public void resetIntegrative() 
    {
        integrative = 0.0;
    }
}