package frc.robot.commands;

import edu.wpi.first.math.MathUtil;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kDrive;
import frc.robot.subsystems.Drivetrain;

public class SwerveDrive extends Command {

    // Controllers
    private final CommandXboxController m_controller;

    // Subsystems
    private final Drivetrain m_drivetrain;

    public SwerveDrive(CommandXboxController controller) {

        // Controllers
        m_controller = controller;

        // Subsystems
        m_drivetrain = Drivetrain.getInstance();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        // Inputs
        double xSpeed = -m_controller.getLeftY();
        double ySpeed = -m_controller.getLeftX();

        double xRot = m_controller.getRightX();
        double yRot = m_controller.getRightY();

        // Deadband
        if (Math.abs(xRot) < kDrive.kTargetHeadingDeadband) xRot = 0;
        if (Math.abs(yRot) < kDrive.kTargetHeadingDeadband) yRot = 0;

        // Angles
        double manualRot = m_controller.getLeftTriggerAxis() + -m_controller.getRightTriggerAxis();
        double targetAngle = getRotTargetAngle(xRot, yRot);

        // Apply deadband & slew rate
        xSpeed = MathUtil.applyDeadband(xSpeed, kDrive.kXSpeedDeadband);
        ySpeed = MathUtil.applyDeadband(ySpeed, kDrive.kYSpeedDeadband);
        manualRot = MathUtil.applyDeadband(manualRot, kDrive.kManualRotationDeadband);

        // Scale from % to speed
        xSpeed *= kDrive.kMaxDriveVelocity; // m/s
        ySpeed *= kDrive.kMaxDriveVelocity; // m/s
        manualRot *= kDrive.kMaxTurnAngularVelocity; // rad

        // Drive!
        m_drivetrain.drive(xSpeed, ySpeed, targetAngle, manualRot);
    }

    /**
     * Get the target rotation angle, in radians.
     * 
     * 0 deg = forward
     * 90 deg = left
     * 180 deg = backward
     * 270 deg = right
     * 
     * @param xRotation right stick x
     * @param yRotation right stick y
     * @return target angle
     */
    private double getRotTargetAngle(double xRot, double yRot) {
        if (xRot == 0 && yRot == 0) {
            return -1;
        }

        double angle = Math.atan2(-xRot, -yRot);
        if (angle < 0) angle += Math.toRadians(360);
        return angle;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}