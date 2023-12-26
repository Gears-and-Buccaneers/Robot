package frc.lib.hardware.sensor.proximitySwitch;

import org.littletonrobotics.junction.LogTable;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.hardware.sensor.SensorRequirments;

public interface ProximitySwitch extends SensorRequirments {
  /**
   * @return true if open false if closed
   */
  boolean get();

  /**
   * @return true if closed false if open
   */
  public default boolean isClosed() {
    return !get();
  }

  /**
   * @return true if open false if closed
   */
  public default boolean isOpen() {
    return get();
  }

  public default Trigger trigger() {
    return new Trigger(
        () -> {
          return isOpen();
        });
  }

  default void toLog(LogTable table) {
    table.put(this.getClass().getSimpleName() + "/Open", get());
  }

  default void fromLog(LogTable table) {
    throw new UnsupportedOperationException("Unimplemented method 'fromLog'");
  }
}