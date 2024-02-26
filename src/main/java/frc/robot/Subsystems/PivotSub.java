package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class PivotSub implements SubsystemReq {
    private final String simpleName = this.getClass().getSimpleName();

    // Hardware
    private TalonFX pivot1;
    private TalonFX pivot2;
    private final DigitalInput limit = new DigitalInput(0);

    // vars
    private double kA = 0.0;
    private String kAKey = simpleName + "kA";
    private double kD = 0.0;
    private String kDKey = simpleName + "kD";
    private double kG = 0.0;
    private String kGKey = simpleName + "kG";
    private double kI = 0.0;
    private String kIKey = simpleName + "kI";
    private double kP = 10.0;
    private String kPKey = simpleName + "kP";
    private double kS = 0.0;
    private String kSKey = simpleName + "kS";
    private double kV = 0.0;
    private String kVKey = simpleName + "kV";
    private double setpoint = -1;
    // private boolean homed = false; // TODO: add auto home

    // Sim
    // units are inches and degrees
    private Mechanism2d mech = new Mechanism2d(28, 28);
    private MechanismRoot2d root = mech.getRoot("PivPoint", 20, 18);
    private MechanismLigament2d mecPivot = root.append(new MechanismLigament2d("Shooter", 18, 180));

    public PivotSub() {
        TalonFXConfiguration conf = new TalonFXConfiguration();

        conf.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO: CHECK THIS
        conf.Feedback.SensorToMechanismRatio = 100; // gear ratio 100:1

        conf.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        conf.MotorOutput.PeakForwardDutyCycle = 1;
        conf.MotorOutput.PeakReverseDutyCycle = -1;

        conf.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        conf.Slot0.kA = kA;
        conf.Slot0.kD = kD;
        conf.Slot0.kG = kG;
        conf.Slot0.kI = kI;
        conf.Slot0.kP = kP;
        conf.Slot0.kS = kS;
        conf.Slot0.kV = kV;

        conf.MotionMagic.MotionMagicAcceleration = 0.2;
        conf.MotionMagic.MotionMagicCruiseVelocity = 0.2;
        // conf.MotionMagic.MotionMagicJerk

        // TODO: CurrentLimit

        pivot1 = new TalonFX(7);
        pivot2 = new TalonFX(17);

        pivot1.getConfigurator().apply(conf);
        pivot2.getConfigurator().apply(conf);

        pivot2.setControl(new Follower(7, true));

        // vars
        Preferences.initDouble(kAKey, kA);
        Preferences.initDouble(kDKey, kD);
        Preferences.initDouble(kGKey, kG);
        Preferences.initDouble(kIKey, kI);
        Preferences.initDouble(kPKey, kP);
        Preferences.initDouble(kSKey, kS);
        Preferences.initDouble(kVKey, kV);

        System.out.println("[Init] Creating " + simpleName + " with:");
        System.out.println("\t" + pivot1.getClass().getSimpleName() + " ID:" + pivot1.getDeviceID());
        System.out.println("\t" + pivot2.getClass().getSimpleName() + " ID:" + pivot2.getDeviceID());
    }

    public Command setPivotAngle(double setpoint) {
        mecPivot.setAngle(pivot1.getPosition().getValueAsDouble() * 3.6);
        this.setpoint = setpoint;

        return new FunctionalCommand(() -> {
            pivot1.setControl(new MotionMagicDutyCycle(setpoint));
        }, () -> {
        }, (_interrupted) -> {
            disable();
        }, () -> Math.abs(pivot1.getPosition().getValueAsDouble() - setpoint) < .005, this);
        // .onlyIf(() -> homed);
    }

    public Command intakePos() {
        return setPivotAngle(0.01);
    }

    public boolean atIntakePos() {
        return Math.abs(pivot1.getPosition().getValueAsDouble() - setpoint) < .008;
    }

    public Command speekerPose(Pose2d robotPose) {
        return setPivotAngle(0.028564453125); // TODO: make this

    }

    public Command ampPose() {
        return setPivotAngle(0.310302734375);
    }

    public void disable() {
        this.setpoint = -1;
        pivot1.set(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        SmartDashboard.putBoolean("Limit switch active", limit.get());

        SmartDashboard.putNumber("pivot Pose", pivot1.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("pivot Velocity", pivot1.getVelocity().asSupplier().get());
        SmartDashboard.putNumber("pivot Acceleration", pivot1.getAcceleration().asSupplier().get());
        SmartDashboard.putNumber("pivot Current", pivot1.getSupplyCurrent().asSupplier().get());
        SmartDashboard.putNumber("pivot Volts", pivot1.getSupplyVoltage().asSupplier().get());
        SmartDashboard.putNumber("pivot Temp", pivot1.getDeviceTemp().asSupplier().get());
        SmartDashboard.putData("pivot Mech", mech);
    }
}
