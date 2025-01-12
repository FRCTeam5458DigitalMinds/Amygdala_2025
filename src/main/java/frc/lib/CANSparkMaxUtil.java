package frc.lib;

import com.revrobotics.spark.*;
/** Sets motor usage for a Spark Max motor controller */
public class CANSparkMaxUtil {
  public enum Usage {
    kAll,
    kPositionOnly,
    kVelocityOnly,
    kMinimal
  };

  /**
   * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 20ms to 500ms.
   *
   * <p>See
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   * for a description of the status frames.
   *
   * @param motor The motor to adjust the status frame periods on.
   * @param usage The status frame feedack to enable. kAll is the default when a CANSparkMax is
   *     constructed.
   * @param enableFollowing Whether to enable motor following.
   */
  public static void setCANSparkMaxBusUsage(
    SparkMax motor, Usage usage, boolean enableFollowing) {
    if (enableFollowing) {
      motor.setControlFramePeriodMs(10);

    } else {
      motor.setControlFramePeriodMs(500);
    }

    if (usage == Usage.kAll) {
      //sets ussage to send all the frames of data yay
      motor.setControlFramePeriodMs(20);
    } else if (usage == Usage.kPositionOnly) {
      //only sends the position frames every 20 ms, saves on velocity and other status
      motor.setControlFramePeriodMs(20);
    } else if (usage == Usage.kVelocityOnly) {
      //only sends the velocity every 20 ms
      motor.setControlFramePeriodMs(20);

    } else if (usage == Usage.kMinimal) {
      //sends as little data as possible to save canbus ussage
      motor.setControlFramePeriodMs(500);
    }
  }

  /**
   * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 20ms to 500ms.
   *
   * <p>See
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   * for a description of the status frames.
   *
   * @param motor The motor to adjust the status frame periods on.
   * @param usage The status frame feedack to enable. kAll is the default when a CANSparkMax is
   *     constructed.
   */
  public static void setCANSparkMaxBusUsage(SparkMax motor, Usage usage) {
    setCANSparkMaxBusUsage(motor, usage, false);
  }
}