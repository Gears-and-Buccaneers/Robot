package frc.hardware;

public interface Motor {
	/** Sets the motor to run at the specific percent from -1.0 to +1.0. */
	public void setPercent(double percent);

	/** Sets the motor to run at the specified voltage. */
	public void setVoltage(double voltage);
}