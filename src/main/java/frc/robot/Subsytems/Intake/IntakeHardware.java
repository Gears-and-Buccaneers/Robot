package frc.robot.Subsytems.Intake;

import org.littletonrobotics.junction.LogTable;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Preferences;
// import frc.lib.hardware.motorController.*;
import frc.lib.hardware.sensor.proximitySwitch.*;

public class IntakeHardware implements IntakeRequirments {
  private final TalonSRX motor;
  private final ProximitySwitch switch1;

  private double setVolts = .1;

  public IntakeHardware() {
    motor = new TalonSRX(10);
    switch1 = new Huchoo(3);

    initPrefrences();
  }

  public void setIntakeVoltage() {
    motor.set(ControlMode.PercentOutput, setVolts);
  }

  public void setOutakeVoltage() {
    motor.set(ControlMode.PercentOutput, -setVolts);
  }

  public void off() {
    motor.set(ControlMode.Disabled, 0);
  }

  public boolean isOpen() {
    return switch1.isOpen();
  }

  public boolean isClosed() {
    return switch1.isClosed();
  }

  // required things-------------------------------------------------

  @Override
  public void close() throws Exception {
    switch1.close();
  }

  private void initPrefrences() {
    Preferences.initDouble("maxVolts", setVolts);
  }

  public void loadPreferences() {
    setVolts = Preferences.getDouble("maxVolts", setVolts);
  }

  @Override
  public void toLog(LogTable table) {
    table.put("SupplyCurrent", motor.getSupplyCurrent());
    table.put("thing", switch1);
  }

  @Override
  public void fromLog(LogTable table) {

  }
}