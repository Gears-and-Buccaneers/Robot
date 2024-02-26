package frc.robot;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Subsystems.DrivetrainSub;

public class Vision extends Thread {
    AprilTagFieldLayout field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    Transform3d[] cameraToRobot = {};

    Matrix<N3, N3> rot = new Matrix<N3, N3>(new SimpleMatrix(new float[3][3]));

    DrivetrainSub consumer;
    final DatagramSocket socket;
    final DatagramPacket packet = new DatagramPacket(new byte[108], 108);

    public Vision(int port, DrivetrainSub consumer) {
        try {
            socket = new DatagramSocket(port);
        } catch (SocketException e) {
            throw new Error("SocketException");
        }

        this.consumer = consumer;
    }

    @Override
    public void run() {
        try {
            socket.receive(packet);
            ByteBuffer buf = ByteBuffer.wrap(packet.getData()).order(ByteOrder.LITTLE_ENDIAN);
            int tag = buf.getInt();

            switch (tag) {
                // timestamp request
                case 0:
                    byte[] out = new byte[4];
                    ByteBuffer.wrap(out).order(ByteOrder.LITTLE_ENDIAN).putLong(RobotController.getFPGATime());
                    packet.setData(out);
                    socket.send(packet);
                    break;
                // apriltag measurement
                case 1:
                    long ts = buf.getLong();
                    int camera = buf.getInt();

                    while (buf.hasRemaining()) {
                        int id = Short.toUnsignedInt(buf.getShort());
                        // int error = Byte.toUnsignedInt(buf.get());

                        for (int row = 0; row < 3; row++)
                            for (int col = 0; col < 3; col++)
                                rot.set(row, col, buf.getFloat());

                        Rotation3d rotation = new Rotation3d(rot);

                        Pose3d cameraToTag = new Pose3d(buf.getFloat(), buf.getFloat(), buf.getFloat(), rotation);
                        Pose3d tagAbsolute = field.getTagPose(id).orElseThrow();

                        Pose3d cameraPose = cameraToTag.relativeTo(tagAbsolute);
                        Pose3d robot = cameraPose.transformBy(cameraToRobot[camera]);

                        consumer.addVisionMeasurement(robot.toPose2d(), ts);
                    }

                    break;
                default:
                    System.err.println("!!! Unexpected message tag " + tag + " !!!");
            }
        } catch (IOException e) {
            System.err.println("!!! IOException !!!");
        }
    }
}
