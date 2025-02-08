package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrive extends SubsystemBase {

    private static SwerveDrive m_instance = null;

    public static SwerveDrive getInstance() {
        if (m_instance == null) {
            m_instance = new SwerveDrive();
        }
        return m_instance;
    }

    private SwerveModule[] modules = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.MOD0_CONSTANTS),
            new SwerveModule(1, SwerveConstants.MOD1_CONSTANTS),
            new SwerveModule(2, SwerveConstants.MOD2_CONSTANTS),
            new SwerveModule(3, SwerveConstants.MOD3_CONSTANTS)
    };
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConstants.MODULE_TRANSLATOIN_METERS);

    public SwerveDrive() {}

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        ChassisSpeeds chassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, SwerveConstants.gyroYaw)
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getModulePosition();
        }
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getModuleState();
        }
        return states;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        if (desiredStates.length != 4) {
            throw new IllegalArgumentException("desiredStates must have length 4");
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_MODULE_SPEED);
        for (SwerveModule mod : modules) {
            mod.setDesiredState(desiredStates[mod.ModuleNumber]);
        }
    }

    @Override
    public void periodic() {}
}