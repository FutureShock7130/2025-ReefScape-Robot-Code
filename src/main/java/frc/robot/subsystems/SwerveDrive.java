package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.IO.ModuleIO;

public class SwerveDrive extends SubsystemBase {

    private static SwerveDrive m_instance = null;

    public static SwerveDrive getInstance(ModuleIO FLModuleIO, ModuleIO FRModuleIO, ModuleIO RLModuleIO, ModuleIO RRModuleIO) {
        if (m_instance == null) {
            m_instance = new SwerveDrive(FLModuleIO, FRModuleIO, RLModuleIO, RRModuleIO);
        }
        return m_instance;
    }

    private SwerveModule[] modules = new SwerveModule[4]; // FL FR RL RR

    private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(SwerveConstants.MODULE_TRANSLATION_METERS);

    public SwerveDrive(ModuleIO FLModuleIO, ModuleIO FRModuleIO, ModuleIO RLModuleIO, ModuleIO RRModuleIO) {
        modules[0] = new SwerveModule(FLModuleIO, 0);
        modules[1] = new SwerveModule(FLModuleIO, 1);
        modules[2] = new SwerveModule(FLModuleIO, 2);
        modules[3] = new SwerveModule(FLModuleIO, 3);
    }

    @Override
    public void periodic() {
        for (var module : modules) {
            module.periodic();
        }

        if(DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }
    }

    public void runVelocity(ChassisSpeeds speeds) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = m_kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConstants.MAX_MODULE_SPEED);

        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
        }
    }

    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    public void stopWithButton() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = SwerveConstants.MODULE_TRANSLATION_METERS[i].getAngle();
        }
        m_kinematics.resetHeadings(headings);
        stop();
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }
}