package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSub implements SubsytemReq {
    private TalonFX pivot;

    private DoubleSubscriber pivotSpeed = NetworkTableInstance.getDefault().getDoubleTopic("pivotSpeed")
            .subscribe(0.4);

    private double setpoint = -1;

    private final DigitalInput limit = new DigitalInput(0);
    private boolean homed = false;

    public PivotSub() {
        TalonFXConfiguration conf = new TalonFXConfiguration();

        // conf.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        conf.Feedback.SensorToMechanismRatio = 100;

        conf.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        conf.Slot0.kG = 0.0;
        conf.Slot0.kP = 10;

        conf.MotionMagic.MotionMagicAcceleration = 0.2;
        conf.MotionMagic.MotionMagicCruiseVelocity = 0.2;

        pivot = new TalonFX(7);
        TalonFX pivot2 = new TalonFX(17);

        pivot.getConfigurator().apply(conf);
        pivot2.getConfigurator().apply(conf);

        pivot2.setControl(new Follower(7, true));
    }

    public void periodic() {
        // if (!homed && limit.get()) {
        // homed = true;
        // pivot.set(0);
        // pivot.setPosition(0);
        // } else if (homed && !isSetpoint)
        // pivot.set(operator.getLeftY() * pivotSpeed.get());

        // double pivotManual = -Math.round(operator.getRightY());

        // if (Math.abs(pivotManual) >= 0.1) {
        // pivot.set(pivotManual * 0.0485);
        // } else {
        double pivotJoystick = -operator.getLeftY() * pivotSpeed.get();

        if (setpoint != -1) {
            if (limit.get() && setpoint == 0)
                pivot.set(0);
        } else {
            if ((pivotJoystick < 0 && !limit.get())
                    || (pivotJoystick > 0 && pivot.getPosition().getValueAsDouble() < 120.0 / 360.0))
                pivot.set(pivotJoystick);
            else
                pivot.set(0);
        }

        if (!homed && limit.get()) {
            pivot.setPosition(0);
            homed = true;
        }
        // }

        SmartDashboard.putNumber("pivotS", pivotJoystick);
        SmartDashboard.putBoolean("Limit switch active", limit.get());

        SmartDashboard.putNumber("pivot Pose", pivot.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("pivot Velocity", pivot.getVelocity().asSupplier().get());
        SmartDashboard.putNumber("pivot Acceleration", pivot.getAcceleration().asSupplier().get());
        SmartDashboard.putNumber("pivot Current", pivot.getSupplyCurrent().asSupplier().get());
        SmartDashboard.putNumber("pivot Volts", pivot.getSupplyVoltage().asSupplier().get());
        SmartDashboard.putNumber("pivot Temp", pivot.getDeviceTemp().asSupplier().get());
        piviott.setAngle(pivot.getPosition().getValueAsDouble() * 3.6);
        SmartDashboard.putData("Piviot Mech", mech);
    }

    Command pivot(double pos) {
        return new FunctionalCommand(() -> {
            setpoint = pos;
            pivot.setControl(new MotionMagicDutyCycle(pos));
        }, () -> {
        }, (_interrupted) -> {
            setpoint = -1;
            pivot.set(0);
        }, () -> Math.abs(pivot.getPosition().getValueAsDouble() - pos) < .005, this);
        // .onlyIf(() -> homed);
    }

    @Override
    public void disable() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'disable'");
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }

    // units are inches
    private Mechanism2d mech = new Mechanism2d(28, 28);
    private MechanismRoot2d root = mech.getRoot("PivPoint", 20, 18);
    private MechanismLigament2d piviott = root.append(new MechanismLigament2d("Shooter", 18, 180));
}
