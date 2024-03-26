package frc.system.vision;

import java.util.EnumSet;
import java.util.Optional;
import java.util.function.Consumer;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.struct.Pose3dStruct;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructEntry;

public class Nt implements Vision {
	Transform3d robotToCamera = new Transform3d(Units.inchesToMeters(13.5 - 0.744844), 0,
			Units.inchesToMeters(7.5 + 1.993), new Rotation3d(0, Units.degreesToRadians(-37.5), 0));

	final Measurement cached = new Measurement();
	final AprilTagFieldLayout tags = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

	@Override
	public void register(Consumer<Measurement> drivetrain) {
		NetworkTableInstance nt = NetworkTableInstance.getDefault();

		NetworkTable table = nt.getTable("Subsystems");

		StructEntry<Pose3d> ntOut = table.getStructTopic("RobotPose", new Pose3dStruct()).getEntry(new Pose3d());

		nt.addListener(table.getTopic("Detections"), EnumSet.of(NetworkTableEvent.Kind.kValueAll),
				(event) -> {
					double[] data = event.valueData.value.getDoubleArray();

					if (data.length % 8 != 0)
						System.err.println("WARN: Invalid NetworkTables AprilTag vision data array " + data.length);

					int n = 0;
					double pX = 0, pY = 0, pZ = 0, rX = 0, rY = 0, rZ = 0;

					for (int i = 0; i + 7 < data.length; i += 8) {
						int tagId = (int) data[i];

						double x = data[i + 1], y = data[i + 2], z = data[i + 3];
						double qw = data[i + 4], qx = data[i + 5], qy = data[i + 6], qz = data[i + 7];

						Translation3d translation = new Translation3d(z, -x, -y);
						Rotation3d rotation = new Rotation3d(new Quaternion(qw, qz, -qx, -qy));

						Transform3d cameraToTag = new Transform3d(translation, rotation);

						Optional<Pose3d> maybeTagPose = tags.getTagPose(tagId);

						if (maybeTagPose.isEmpty()) {
							System.err.println("WARN: Unknown tag id " + tagId + "detected");
							break;
						}

						Pose3d fieldTagPose = maybeTagPose.get();
						Pose3d fieldTagPoseFlipped = new Pose3d(fieldTagPose.getTranslation(),
								fieldTagPose.getRotation().rotateBy(new Rotation3d(0, 0, Math.PI)));

						Transform3d robotToTag = robotToCamera.plus(cameraToTag);
						Pose3d robotPose = fieldTagPoseFlipped.transformBy(robotToTag.inverse());

						n++;

						pX += robotPose.getX();
						pY += robotPose.getY();
						pZ += robotPose.getZ();

						rX += robotPose.getRotation().getX();
						rY += robotPose.getRotation().getY();
						rZ += robotPose.getRotation().getZ();
					}

					if (n != 0) {

						cached.pose = new Pose3d(pX / n, pY / n, pZ / n, new Rotation3d(rX / n, rY / n, rZ / n));
						cached.timestamp = event.valueData.value.getTime();  // Timer.getFPGATimestamp() (I Think this needs to be FPGA Timestamp to make sense ref: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/mechanisms/swerve/SwerveDrivetrain.html addVisionMeasurement )
						// TODO: calculate standard deviations of pose drivetrain. This TODO needs to happen as we're getting significantly more measurements than the avg FRC team which is part of why the Kalman filter is freaking out... 
						cached.stdDev = null;

						drivetrain.accept(cached);

						ntOut.set(cached.pose);
					}
				});
	}
}
