package frc.FSLib2025.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConfig {

    public int DriveMotorId;
    public int AngleMotorId;
    public int CancoderId;
    public Rotation2d AngleOffset;
    
    public SwerveModuleConfig (int driveMotorId, int angleMotorId, int cancoderId, Rotation2d angleOffset) {
        this.DriveMotorId = driveMotorId;
        this.AngleMotorId = angleMotorId;
        this.CancoderId = cancoderId;
        this.AngleOffset = angleOffset;
    }
}
