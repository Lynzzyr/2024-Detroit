package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kDrive;
import frc.robot.Constants.kDrive.kLocation;
import frc.robot.Constants.kDrive.kPID;
import frc.robot.Constants.kDrive.kRelativeEncoder;

/**
 * SDS MK4i
 * REV NEO
 * L2 Ratio
 */
public class SwerveModule extends SubsystemBase {
    private static SwerveModule instance = null;

    // Motors
    private final CANSparkMax motDrive;
    private final CANSparkMax motTurn;

    // Encoders
    private final RelativeEncoder encDrive;
    private final RelativeEncoder encTurn;
    private final CANcoder m_CANcoder;
    private final MagnetSensorConfigs m_CANcoderMagnetConfig;

    // PID
    private final SparkMaxPIDController pidMotDrive;
    private final SparkMaxPIDController pidMotTurn;

    // Location
    private final kLocation m_location;
    private boolean isStuck = false;

    public SwerveModule(
        int motDriveID,
        int motTurnID,
        int CANcoderID,
        double CANcoderAbsOffset,
        boolean motDriveInverted,
        boolean motTurnInverted,
        kLocation location
    ) {
        // Motors
        motDrive = new CANSparkMax(motDriveID, MotorType.kBrushless);
        motTurn = new CANSparkMax(motTurnID, MotorType.kBrushless);

        // Encoders
        encDrive = motDrive.getEncoder();
        encTurn = motTurn.getEncoder();
        m_CANcoder = new CANcoder(CANcoderID);
        m_CANcoderMagnetConfig = new MagnetSensorConfigs();

        // PID
        pidMotDrive = motDrive.getPIDController();
        pidMotTurn = motTurn.getPIDController();

        // Location
        m_location = location;

        // Config
        configAll(motDriveInverted, motTurnInverted, -CANcoderAbsOffset);
    }

    // Get subsystem
    public static SwerveModule getInstance(int motDriveID, int motTurnID, int CANcoderID, double CANcoderAbsOffset, boolean motDriveInverted, boolean motTurnInverted, kLocation location) {
        if (instance == null) instance = new SwerveModule(motDriveID, motTurnID, CANcoderID, CANcoderAbsOffset, motDriveInverted, motTurnInverted, location);
        return instance;
    }

    private void configAll(boolean motDriveInverted, boolean motTurnInverted, double CANcoderAbsOffset) {
        // Motors
        motDrive.restoreFactoryDefaults();
        motDrive.setInverted(motDriveInverted);
        motDrive.setSmartCurrentLimit(kDrive.kDriveMotorCurrentLimit);
        setRampRate(true);
        pidMotDrive.setP(kPID.kDriveP);
        pidMotDrive.setI(kPID.kDriveI);
        pidMotDrive.setD(kPID.kDriveD);
        pidMotDrive.setFF(kPID.kDriveFF);

        motTurn.restoreFactoryDefaults();
        motTurn.setInverted(motTurnInverted);
        motTurn.setSmartCurrentLimit(kDrive.kTurnMotorCurrentLimit);
        pidMotTurn.setP(kPID.kTurnP);
        pidMotTurn.setI(kPID.kTurnI);
        pidMotTurn.setD(kPID.kTurnD);
        pidMotTurn.setFF(kPID.kTurnFF);
        pidMotTurn.setSmartMotionMaxAccel(kDrive.kMaxTurnAngularAcceleration, 0);
        pidMotTurn.setPositionPIDWrappingEnabled(true);
        pidMotTurn.setPositionPIDWrappingMaxInput(2 * Math.PI);
        pidMotTurn.setPositionPIDWrappingMinInput(0);

        setBrakeMode(false); // will be set true later

        // Encoders
        encDrive.setVelocityConversionFactor(kRelativeEncoder.kDriveSensorCoefficient / 60);
        encDrive.setPositionConversionFactor(kRelativeEncoder.kTurnSensorCoefficient);

        encTurn.setVelocityConversionFactor(kRelativeEncoder.kTurnSensorCoefficient / 60);
        encTurn.setPositionConversionFactor(kRelativeEncoder.kTurnSensorCoefficient);

        m_CANcoderMagnetConfig.MagnetOffset = CANcoderAbsOffset / 360; // CANcoderAbsOffset is degrees and Phoenix v6 now uses rotations not degrees
        m_CANcoderMagnetConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        m_CANcoder.getConfigurator().apply(m_CANcoderMagnetConfig);

        motDrive.burnFlash();
        motTurn.burnFlash();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(encDrive.getVelocity(), new Rotation2d(encTurn.getPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(encDrive.getPosition(), new Rotation2d(encTurn.getPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            stopMotors();
            return;
        }

        SwerveModuleState optimizedState = optimize(desiredState, new Rotation2d(encTurn.getPosition()));

        pidMotDrive.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity);
        pidMotTurn.setReference(optimizedState.angle.getRadians(), ControlType.kPosition);
    }

    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        Rotation2d delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > 120) {
            return new SwerveModuleState(-desiredState.speedMetersPerSecond, desiredState.angle.rotateBy(Rotation2d.fromDegrees(180)));
        } else {
            return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
    }

    public void stopMotors() {
        motDrive.set(0);
        motTurn.set(0);
    }

    public CANSparkMax getMotDrive() {
        return motDrive;
    }

    public void setBrakeMode(boolean brakeMode) {
        motDrive.setIdleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
        motTurn.setIdleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public boolean getBrakeMode() {
        return (motDrive.getIdleMode() == IdleMode.kBrake ? true : false);
    }

    public void setRampRate(boolean enable) {
        motDrive.setClosedLoopRampRate(enable ? kDrive.kDriveRampRate : 0);
    }

    public boolean getRampRate() {
        return (motDrive.getClosedLoopRampRate() > 0);
    }

    public void setMotTurnDegrees(double degrees) {
        pidMotTurn.setReference(Math.toRadians(degrees), ControlType.kPosition);
    }

    public void setMotTurnRadians(double radians) {
        pidMotTurn.setReference(radians, ControlType.kPosition);
    }

    public void resetEncoders() {
        encDrive.setPosition(0);
        encTurn.setPosition(getAbsEncoderTurnPos(true));
    }

    public double getEncoderTurnPos() {
        return encTurn.getPosition();
    }

    public double getAbsEncoderTurnPos(boolean returnRadians) {
        return (returnRadians ? Math.toRadians(encTurn.getPosition()) : encTurn.getPosition());
    }

    public void setTurnStuckState(boolean stuck) {
        isStuck = stuck;
    }

    public boolean getTurnStuckState() {
        return isStuck;
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}
}