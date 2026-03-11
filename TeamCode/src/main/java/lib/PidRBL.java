/******************************************************************************* 
 * 
 * File        : PidRBL.java (v2.0)
 * Library     : LyonLibFTC (from FRC)
 * Description : Advanced PID controller class implementing 
 *               Proportional-Integral-Derivative control with optional 
 *               Feedforward. Supports real-time update, input/output clamping, 
 *               tolerance checking, and continuous inputs (e.g. angle wrap-around).
 * 
 * Authors     : AKA (2024), last update by AKA (2025) 
 *                                and inspired by Team 1678
 * Organization: Robo'Lyon - FTC Teams
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/FTC-RoboLyon
 * 
 *******************************************************************************/

package lib;

public class PidRBL {
    private static final double THEORETICAL_DT = 0.02; // 20 ms par défaut FRC

    private double kp;
    private double ki;
    private double kd;
    private double feedforward;

    private double outputMin = -1.0;
    private double outputMax = 1.0;
    private double inputMin;
    private double inputMax;

    private boolean isContinuous = false;
    private boolean isInputLimitsActive = false;

    private double previousError = 0.0;
    private double integrative = 0.0;
    private double setpoint = 0.0;
    private double currentError = 0.0;

    private double output = 0.0;
    private double tolerance = 0.0;

    private double lastTimestamp = 0.0;
    private double dt = THEORETICAL_DT;

    // ----- Constructors -----
    public PidRBL() {
        this(0, 0, 0, 0);
    }

    public PidRBL(double kp, double ki, double kd) {
        this(kp, ki, kd, 0.0);
    }

    public PidRBL(double kp, double ki, double kd, double ff) {
        SetGains(kp, ki, kd, ff);
    }

    // ----- Configuration -----
    public void SetGains(double kp, double ki, double kd, double ff) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.feedforward = ff;
        Reset();
    }

    public void SetFeedforward(double ff) {
        this.feedforward = ff;
    }

    public void SetSetpoint(double setpoint) {
        if (isInputLimitsActive) {
            this.setpoint = clamp(setpoint, inputMin, inputMax);
        } else {
            this.setpoint = setpoint;
        }
    }

    public void SetTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public void SetOutputLimits(double min, double max) {
        this.outputMin = min;
        this.outputMax = max;
    }

    public void SetInputLimits(double min, double max) {
        this.inputMin = min;
        this.inputMax = max;
        this.isInputLimitsActive = true;
    }

    public void SetInputLimits(boolean active) {
        this.isInputLimitsActive = active;
    }

    public void SetContinuous(boolean isContinuous) {
        this.isContinuous = isContinuous;
    }

    // ----- Getters -----
    public double GetKP() { return kp; }
    public double GetKI() { return ki; }
    public double GetKD() { return kd; }
    public double GetFF() { return feedforward; }
    public double GetError() { return currentError; }
    public double GetSetpoint() { return setpoint; }
    public String GetState() {
        return String.format("PID[kp=%.3f, ki=%.3f, kd=%.3f, ff=%.3f, setpoint=%.3f, error=%.3f, output=%.3f]",
                kp, ki, kd, feedforward, setpoint, currentError, output);
    }

    // ----- Calculations -----
    public double Calculate(double measurement) {
        return Calculate(setpoint, measurement);
    }

    public double Calculate(double setpoint, double measurement) {
        SetSetpoint(setpoint);
        return calculateInternal(measurement, dt);
    }

    public double CalculateWithRealTime(double measurement, double timestamp) {
        double realDt = timestamp - lastTimestamp;
        if (lastTimestamp == 0.0) {
            realDt = THEORETICAL_DT; // première exécution
        }
        lastTimestamp = timestamp;
        return calculateInternal(measurement, realDt);
    }

    public double CalculateWithRealTime(double setpoint, double measurement, double timestamp) {
        SetSetpoint(setpoint);
        return CalculateWithRealTime(measurement, timestamp);
    }

    private double calculateInternal(double measurement, double deltaTime) {
        currentError = setpoint - measurement;

        if (isContinuous) {
            double range = inputMax - inputMin;
            if (Math.abs(currentError) > range / 2.0) {
                if (currentError > 0) {
                    currentError = currentError - range;
                } else {
                    currentError = currentError + range;
                }
            }
        }

        // Anti-windup: accumulate integrative error only if P-term is within output bounds
        if((currentError * kp) < outputMax && (currentError * kp) > outputMin)
        {
            integrative += currentError * deltaTime;
        }
        else 
        {
            integrative = 0.0;
        }

         // If error is above tolerance, calculate full PID output
        if(Math.abs(currentError) >= tolerance) 
        {
            output =  kp * currentError + 
                    ki * integrative + 
                    kd * ((currentError - previousError) / deltaTime) + 
                    feedforward;
        } 
        else
        {
            // Near target: skip proportional term to reduce overshoot
            output = ki * integrative + 
                    kd * ((currentError - previousError) / deltaTime) + 
                    feedforward;
        }
        previousError = currentError;
        return clamp(output, outputMin, outputMax);
    }

    // ----- Resets -----
    public void Reset() {
        setpoint = setpoint - currentError; // hold current position
        previousError = 0.0;
        currentError = 0.0;
        output = 0.0;
        ResetIntegrative();
    }

    public void Reset(double timestamp) {
        Reset();
        lastTimestamp = timestamp;
    }

    public void ResetIntegrative() {
        integrative = 0.0;
    }

    // ----- State -----
    public boolean AtSetpoint() {
        return Math.abs(currentError) <= tolerance;
    }

    // ----- Utils -----
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}