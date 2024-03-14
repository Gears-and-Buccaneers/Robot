package frc.robot.Subsystems.Drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.joysticks.Driver;
import org.littletonrobotics.junction.Logger;

public class DrivetrainSub extends SubsystemBase implements AutoCloseable {
  public final DrivetrainReq drivetrain;
  public final PoseEstimatorReq poseEstimator;

  private final String simpleName = this.getClass().getSimpleName();

  public DrivetrainSub(DrivetrainReq drivetrain, PoseEstimatorReq poseEstimator) {
    this.drivetrain = drivetrain;
    this.poseEstimator = poseEstimator;

    AutoBuilder.configureHolonomic(
        this.poseEstimator::getPose2d, // chaing to pose estimate
        this.poseEstimator::resetEstimator, // chaing to pose estimate
        this.drivetrain::getChassisSpeed,
        this.drivetrain::setChassisSpeed,
        new HolonomicPathFollowerConfig(
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(5.0, 0.0, 0.0),
            this.drivetrain.getMaxModuleSpeed(),
            this.drivetrain.getRadius(),
            new ReplanningConfig()),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);

    System.out.println("[Init] Creating " + simpleName + " with:");
    System.out.println("\t" + this.drivetrain.getClass().getSimpleName());
    // System.out.println("\t" + this.poseEstimator.getClass().getSimpleName());

    this.drivetrain.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    Logger.processInputs(simpleName, drivetrain);
    // Logger.processInputs(simpleName + "/poseEstimator", poseEstimator);

    drivetrain.periodic();
    // poseEstimator.periodic();
  }

  @Override
  public void simulationPeriodic() {}

  // Commands ---------------------------------------------------------
  public Command drive(Driver controller) {
    return run(
        () -> {
          drivetrain.drive(
              controller.getDrivetrainTranslationX(),
              controller.getDrivetrainTranslationY(),
              controller.getDrivetrainRotation());
        });
  }

  public Command goToPose(Pose2d pose) {
    return run(
        () -> {
          // AutoBuilder.pathfindToPose(
          // pose,
          // new PathConstraints(
          // drivetrain.getMaxModuleSpeed(),
          // drivetrain.getMaxModuleAccl(),
          // drivetrain.getMaxAngularVelocity(),
          // drivetrain.getMaxAngularAccl()),
          // 0);
          drivetrain.setChassisSpeed(
              ChassisSpeeds.fromFieldRelativeSpeeds(2, 0, 0, drivetrain.getAngle()));
        });
  }

  // ---------------------------------------------------------
  @Override
  public void close() throws Exception {
    drivetrain.close();
  }
}
