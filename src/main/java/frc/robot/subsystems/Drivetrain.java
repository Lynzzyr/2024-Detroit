package frc.robot.subsystems;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    private static Drivetrain instance = null;

    private Drivetrain() {
        
    }

    // Get subsystem
    public static Drivetrain getInstance() {
        if (instance == null) instance = new Drivetrain();

        return instance;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

}