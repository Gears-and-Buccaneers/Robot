package frc;

import edu.wpi.first.wpilibj.Preferences;
import frc.config.MecanumConfig;
import frc.config.SwerveConfig;
import frc.system.Drivetrain;

public interface Config {
	public static Config get() {
		String bot = Preferences.getString("bot", null);
		switch (bot) {
			case "swerve":
				return new SwerveConfig();
			case "mecanum":
				return new MecanumConfig();
			default:
				throw new Error("Invalid robot ID '" + bot + "'!");
		}
	}

	/** Get an instance of Drivetrain. */
	Drivetrain drivetrain();
}