/*******************************************************************************
 * 
 * File        : FeedForwardModel.java (v2.0)
 * Library     : LyonLib - FTC edition
 * Description : Implements a Feedforward control model used to compute the
 *               voltage required to achieve a desired motion of a mechanism.
 *               The model supports static friction, velocity, acceleration,
 *               and optional gravity compensation.
 * 
 * Authors     : AKA (2026), last update by AKA (06/2026)
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/

package org.firstinspires.ftc.teamcode.Lib.LyonLib.control;

import org.firstinspires.ftc.teamcode.Lib.LyonLib.logging.DebugUtils;
import org.firstinspires.ftc.teamcode.Lib.LyonLib.utils.UtilsRBL;

import java.util.function.DoubleUnaryOperator;

/**
 * FeedForwardModel class
 *
 * <p>This class implements a feedforward controller commonly used in robotics
 * to estimate the voltage required to achieve a desired motion.
 *
 * <p>The model follows the equation:
 *
 * V = kS * sign(v) + kV * v + kA * a + kG(position)
 *
 * <ul>
 * <li>kS : static friction compensation</li>
 * <li>kV : velocity proportional gain</li>
 * <li>kA : acceleration proportional gain</li>
 * <li>kG : optional gravity compensation</li>
 * </ul>
 */
public class FeedForwardModel 
{

  private double kS; // static friction gain (V)
  private double kV; // velocity gain (V per velocity unit)
  private double kA; // acceleration gain (V per acceleration unit)

  /** Gravity compensation function */
  private DoubleUnaryOperator kG;

  private double outputMin = -1.0;
  private double outputMax = 1.0;

  private double output = 0.0;

  /** Default constructor (all gains set to zero). */
  public FeedForwardModel() 
  {
    setGains(0.0, 0.0, 0.0, position -> 0.0);
  }

  /**
   * Constructor with gains.
   *
   * @param kS static friction gain
   * @param kV velocity gain
   * @param kA acceleration gain
   */
  public FeedForwardModel(double kS, double kV, double kA) 
  {
    setGains(kS, kV, kA, position -> 0.0);
  }

  /**
   * Constructor with gains and gravity compensation.
   *
   * @param kS static friction gain
   * @param kV velocity gain
   * @param kA acceleration gain
   * @param kG gravity compensation function
   */
  public FeedForwardModel(double kS, double kV, double kA, DoubleUnaryOperator kG) 
  {
    setGains(kS, kV, kA, kG);
  }

  /**
   * Sets the Feedforward gains for the controller.
   *
   * @param kS Static friction gain
   * @param kV Velocity gain
   * @param kA Acceleration gain
   * @param kG Gravity compensation function
   */
  public void setGains(double kS, double kV, double kA, DoubleUnaryOperator kG) 
  {
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    this.kG = (kG != null) ? kG : (p -> 0.0);
  }

  /**
   * Sets the Feedforward gains without gravity compensation.
   */
  public void setGains(double kS, double kV, double kA) 
  {
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
  }

  /**
   * Sets the output limits for the Feedforward controller.
   *
   * @param min minimum output value
   * @param max maximum output value
   */
  public void setOutputLimits(double min, double max) 
  {

    outputMin = min;
    outputMax = max;

    if (DebugUtils.assertion((outputMin < outputMax), "FeedForwardModel: Output minimum must be < output maximum.")) 
    {
        // Ensure min and max are in the correct order in competition mode (where we don't throw an exception)
        outputMin = Math.min(min, max);
        outputMax = Math.max(min, max);
    }
  }

  public double getKS() 
  {
    return kS;
  }

  public double getKV() 
  {
    return kV;
  }

  public double getKA() 
  {
    return kA;
  }

  public double getKG(double position) 
  {
    return kG.applyAsDouble(position);
  }

  /**
   * Returns the internal state of the FeedForward model as a formatted string.
   */
  public String getState() 
  {
    StringBuilder state = new StringBuilder("FeedForwardModel State:\n");

    state.append("KS: ").append(kS).append("\n");
    state.append("KV: ").append(kV).append("\n");
    state.append("KA: ").append(kA).append("\n");

    if (kG != null) 
    {
      state.append("KG(0.0): ").append(kG.applyAsDouble(0.0)).append("\n");
    } else 
    {
      state.append("KG: <none>\n");
    }

    state.append("Output: ").append(output).append("\n");
    state.append("Output Min: ").append(outputMin).append("\n");
    state.append("Output Max: ").append(outputMax).append("\n");

    return state.toString();
  }

  /**
   * Calculates the Feedforward output.
   *
   * @param position desired mechanism position
   * @param velocity desired velocity
   * @param acceleration desired acceleration
   * @return feedforward output in output units (duty cycle if output limits are not set)
   */
  public double calculate(double position, double velocity, double acceleration) 
  {

    output =
        kS * Math.signum(velocity)
            + kV * velocity
            + kA * acceleration
            + kG.applyAsDouble(position);

    return UtilsRBL.clamp(outputMin, output, outputMax);
  }

  /**
   * Calculates the feed-forward output.
   * 
   * This convenience overload delegates to the three-argument calculate method
   * using a default position of 0.0.
   *
   * @param velocity desired velocity
   * @param acceleration desired acceleration
   * @return the feed-forward output corresponding to the specified velocity and acceleration
   */
  public double calculate(double velocity, double acceleration) 
  {
    return calculate(0.0, velocity, acceleration);
  }

  /**
   * Calculates the feed-forward output.
   * 
   * This convenience overload delegates to the three-argument calculate method
   * using a default position of 0.0 and a default acceleration of 0.0.
   *
   * @param velocity desired velocity
   * @return the feed-forward output corresponding to the specified velocity
   */
  public double calculate(double velocity) 
  {
    return calculate(0.0, velocity, 0.0);
  }
}