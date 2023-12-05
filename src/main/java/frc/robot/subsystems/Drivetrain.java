package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kDrivetrain;
import frc.robot.Constants.kDrivetrain.Location;
import frc.robot.Constants.kDrivetrain.kAutonomous;
import frc.robot.Constants.kDrivetrain.kCANID;
import frc.robot.Constants.kDrivetrain.kCANcoder;
import frc.robot.Constants.kDrivetrain.kPID;
import frc.robot.Constants.kDrivetrain.kRobot;

public class Drivetrain extends SubsystemBase {

    private static Drivetrain instance = null;

    // Modules
    private final SwerveModule modFL;
    private final SwerveModule modFR;
    private final SwerveModule modBL;
    private final SwerveModule modBR;

    // Kinematic points
    private final Translation2d modLocFL;
    private final Translation2d modLocFR;
    private final Translation2d modLocBL;
    private final Translation2d modLocBR;

    // Sensors
    private final Pigeon2 gyro;
    private final MountPoseConfigs gyroMountPosConfig;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    // PID
    private final PIDController headingPID;
    private double headingPIDSetpoint = -1; // initial of -1 for null setpoint

    private Drivetrain() {
        // Modules
        modFL = SwerveModule.getInstance(kCANID.kMotDriveFL, kCANID.kMotTurnFL, kCANID.kCANcoderFL, kCANcoder.kAbsOffsetFL, false, true, Location.FRONT_LEFT);
        modFR = SwerveModule.getInstance(kCANID.kMotDriveFR, kCANID.kMotTurnFR, kCANID.kCANcoderFR, kCANcoder.kAbsOffsetFR, true, true, Location.FRONT_RIGHT);
        modBL = SwerveModule.getInstance(kCANID.kMotDriveBL, kCANID.kMotTurnBL, kCANID.kCANcoderBL, kCANcoder.kAbsOffsetBL, false, true, Location.BACK_LEFT);
        modBR = SwerveModule.getInstance(kCANID.kMotDriveBR, kCANID.kMotTurnBR, kCANID.kCANcoderBR, kCANcoder.kAbsOffsetBR, true, true, Location.BACK_RIGHT);

        // Kinematic points
        modLocFL = new Translation2d(kRobot.kLength / 2, kRobot.kWidth / 2);
        modLocFR = new Translation2d(kRobot.kLength / 2, -kRobot.kWidth / 2);
        modLocBL = new Translation2d(-kRobot.kLength / 2, kRobot.kWidth / 2);
        modLocBR = new Translation2d(-kRobot.kLength / 2, -kRobot.kWidth / 2);

        // Sensors
        gyro = new Pigeon2(kCANID.kPigeon);
        gyroMountPosConfig = new MountPoseConfigs();
        gyroMountPosConfig.MountPoseYaw = kCANcoder.kMountPoseYaw;
        gyro.getConfigurator().apply(gyroMountPosConfig);
        zeroHeading();

        kinematics = new SwerveDriveKinematics(new Translation2d[]{modLocFL, modLocFR, modLocBL, modLocBR});
        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), new SwerveModulePosition[]{modFL.getPosition(), modFR.getPosition(), modBL.getPosition(), modBR.getPosition()});

        // PID
        headingPID = new PIDController(kPID.kHeadingP, kPID.kHeadingI, kPID.kHeadingD);
        headingPID.setTolerance(Math.toRadians(3));
        headingPID.enableContinuousInput(0, Math.toRadians(360));

        // PathPlanner
        AutoBuilder.configureHolonomic(
            this::getPose2d,
            this::resetOdometry,
            this::getChassisSpeeds,
            this::driveFromChassisSpeeds,
            new HolonomicPathFollowerConfig(kAutonomous.kMaxDriveVelocity, 0.4, new ReplanningConfig()),
            this
        );
    }

    // Get subsystem
    public static Drivetrain getInstance() {
        if (instance == null) instance = new Drivetrain();

        return instance;
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    public Pose2d getPose2d() {
        return new Pose2d(odometry.getPoseMeters().getTranslation(), gyro.getRotation2d());
    }

    public double getHeading() {
        double heading = getRotation2d().getDegrees() % 360;
        if (heading < 0) heading += 360;
        return heading;
    }
    
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {modFL.getPosition(), modFR.getPosition(), modBL.getPosition(), modBR.getPosition()};
    }

    public void updateOdometry() {
        odometry.update(getRotation2d(), getModulePositions());
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    /**
     * Tele-operated drive method
     * 
     * @param xSpeed forward/backward axis, relative to field
     * @param ySpeed left/right axis, relative to field
     * @param targetAngle target heading
     * @param manualRotation manual rotation using triggers
     */
    public void drive(double xSpeed, double ySpeed, double targetAngle, double manualRotation) {
        double currentAngle = Math.toRadians(getHeading());
        double rotation = manualRotation; // manual controls prioritized
        headingPIDSetpoint = (targetAngle == -1) ? headingPIDSetpoint : targetAngle;

        if (rotation == 0) {
            if (headingPIDSetpoint == -1) rotation = 0;
            else {
                headingPID.setSetpoint(headingPIDSetpoint);
                if (Math.abs(headingPIDSetpoint - currentAngle) > headingPID.getPositionTolerance()) {
                    rotation = headingPID.calculate(currentAngle);
                } else headingPIDSetpoint = -1;
            }
        }

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, manualRotation, getRotation2d());
        
    }

    private void driveFromChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kDrivetrain.kMaxDriveVelocity);

        setModuleStates(swerveModuleStates);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        modFL.setDesiredState(states[0]);
        modFR.setDesiredState(states[1]);
        modBL.setDesiredState(states[2]);
        modBR.setDesiredState(states[3]);
    }

    public void resetAllEncoders() {
        modFL.resetEncoders();
        modFR.resetEncoders();
        modBL.resetEncoders();
        modBR.resetEncoders();
    }

    public void setBrakeMode(boolean brakeMode) {
        modFL.setBrakeMode(brakeMode);
        modFR.setBrakeMode(brakeMode);
        modBL.setBrakeMode(brakeMode);
        modBR.setBrakeMode(brakeMode);
    }

    public boolean getBrakeMode() {
        try {
            return modFL.getBrakeMode();
        } catch (Exception e) {
            e.printStackTrace();
            return modFR.getBrakeMode();
        }
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(modFL.getState(), modFR.getState(), modBL.getState(), modBR.getState());
    }

    public void stopMotors() {
        modFL.stopMotors();
        modFR.stopMotors();
        modBL.stopMotors();
        modBR.stopMotors();
    }

    public void setRampRate() {
        modFL.setRampRate(true);
        modFR.setRampRate(true);
        modBL.setRampRate(true);
        modBR.setRampRate(true);
    }

    public boolean getRampRate() {
        return modFL.getBrakeMode()
            || modFR.getBrakeMode()
            || modBL.getBrakeMode()
            || modBR.getBrakeMode();
    }

    public SwerveModule[] getModules() {
        SwerveModule[] modules = new SwerveModule[4];

        modules[0] = modFL;
        modules[1] = modFR;
        modules[2] = modBL;
        modules[3] = modBR;

        return modules;
    }

    @Override
    public void periodic() {
        updateOdometry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

}