package frc.robot.Subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SubsytemReq extends Subsystem, Sendable {
    public void disable();

    default void loadPreferences() {
    }
}
