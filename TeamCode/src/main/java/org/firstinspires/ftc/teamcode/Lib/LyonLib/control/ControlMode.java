package org.firstinspires.ftc.teamcode.Lib.LyonLib.control;
    public enum ControlMode {
        // ----- High-level control -----
        PROFILED_PID,               // Motion profiling + PID (trapezoidal, S-curve)
        MOTION_PROFILING,           // Feedforward motion profiling only (open-loop trajectory)

        // ----- Closed-loop control (PID + optional feedforward) -----
        POSITION_VOLTAGE_PID,       // Position control (PID + volts output)
        POSITION_DUTYCYCLE_PID,     // Position control (PID + duty cycle output)
        VELOCITY_VOLTAGE_PID,       // Velocity control (PID + volts output)
        VELOCITY_VOLTAGE_PIDF,      // A control using feedback kp, ki, kd and a feedforward kv, ks, ka
        VELOCITY_DUTYCYCLE_PID,     // Velocity control (PID + duty cycle output)
        MODEL_CONTROLLED,           // Model-based control (dynamic system model + PID feedback)

        // ----- Open-loop control (feedforward or direct) -----
        VELOCITY_VOLTAGE_FF,        // Open-loop velocity control (kS/kV/kA model, output in volts)
        VELOCITY_DUTYCYCLE_FF,      // Open-loop velocity control (kS/kV/kA model, duty cycle output)
        VOLTAGE,                    // Direct voltage control
        DUTY_CYCLE,                 // Direct duty cycle control
        CURRENT,                    // Current control (amps or % of max amps, if stable model available)
        TORQUE,                     // Torque control (if stable model available)

        // ----- Manual / Bypass modes (no state machine) -----
        MANUAL_POSITION,            // Manual position or velocity command with PID
        MANUAL_VOLTAGE,             // Manual voltage command
        MANUAL_VELOCITY,            // Manual velocity command (PID or open-loop)
        MANUAL_DUTY_CYCLE,          // Manual duty cycle command

        // ----- Disabled / Safe mode -----
        DISABLED;                   // Controller output disabled

        // ---------- Utility checks (equivalent to macros) ----------

    public boolean allowsStateMachine() {
        return switch (this) {
            case PROFILED_PID, MOTION_PROFILING, POSITION_VOLTAGE_PID, POSITION_DUTYCYCLE_PID,
                 VELOCITY_VOLTAGE_PID, VELOCITY_DUTYCYCLE_PID, MODEL_CONTROLLED,
                 VELOCITY_VOLTAGE_FF, VELOCITY_DUTYCYCLE_FF, VOLTAGE, DUTY_CYCLE, CURRENT, TORQUE ->
                    true;
            default -> false;
        };
    }

    public boolean bypassStateMachine() {
        return switch (this) {
            case MANUAL_POSITION, MANUAL_VOLTAGE,
                 MANUAL_VELOCITY, MANUAL_DUTY_CYCLE,
                 DISABLED -> true;
            default -> false;
        };
    }

    public boolean isPID() {
        return switch (this) {
            case PROFILED_PID, POSITION_VOLTAGE_PID, POSITION_DUTYCYCLE_PID,
                 VELOCITY_VOLTAGE_PID, VELOCITY_DUTYCYCLE_PID,
                 MANUAL_POSITION, MODEL_CONTROLLED, MANUAL_VELOCITY -> true;
            default -> false;
        };
    }

    public boolean isProfiling() {
        return this == PROFILED_PID || this == MOTION_PROFILING;
    }

    public boolean isVoltageOutputMode() {
        return switch (this) {
            case POSITION_VOLTAGE_PID, VELOCITY_VOLTAGE_PID,
                 VOLTAGE, VELOCITY_VOLTAGE_FF, MANUAL_VOLTAGE -> true;
            default -> false;
        };
    }

    public boolean isDutyCycleOutputMode() {
        return switch (this) {
            case POSITION_DUTYCYCLE_PID, VELOCITY_DUTYCYCLE_PID,
                 VELOCITY_DUTYCYCLE_FF, DUTY_CYCLE, MANUAL_DUTY_CYCLE -> true;
            default -> false;
        };
    }

        public boolean isDisabledMode() {
            return this == DISABLED;
        }

        @Override
        public String toString() {
            return this.name();
        }
    }

