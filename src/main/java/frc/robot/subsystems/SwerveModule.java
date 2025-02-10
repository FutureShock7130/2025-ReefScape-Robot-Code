// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;
import frc.robot.IO.ModuleIO;
import frc.robot.IO.ModuleIO.ModuleIOInputs;

/** Add your docs here. */
public class SwerveModule {
    private static final double WHEEL_RADIUS = SwerveConstants.DRIVE_WHEEL_DIAMETERS / 2; // meters

    private final ModuleIO io;
    private final ModuleIOInputs inputs = new ModuleIOInputs();
    private final int index;

    private final SimpleMotorFeedforward driveFeedForward;
    private final PIDController driveFeedback;
    private final PIDController turnFeedback;
    private Rotation2d angleSetpoint = null;
    private Double speedSetpoint = null;
    private Rotation2d turnRelativeOffset = null;

    public SwerveModule(ModuleIO io, int index) {
        this.io = io;
        this.index = index;

        driveFeedForward = new SimpleMotorFeedforward(SwerveConstants.DRIVE_MOTOR_KS, SwerveConstants.DRIVE_MOTOR_KV, SwerveConstants.DRIVE_MOTOR_KA);
        driveFeedback = new PIDController(SwerveConstants.DRIVE_MOTOR_KP, SwerveConstants.DRIVE_MOTOR_KI, SwerveConstants.DRIVE_MOTOR_KD);
        turnFeedback = new PIDController(SwerveConstants.STEER_MOTOR_KP, SwerveConstants.STEER_MOTOR_KI, SwerveConstants.STEER_MOTOR_KD);

        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
        setBrakeMode(true);
    }

    public void periodic() {
        io.updateInputs(inputs);
        
        if (turnRelativeOffset == null && inputs.turnAbsolutePosition.getRadians() != 0.0) {
            turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
        }

        if (angleSetpoint != null) {
            io.setTurnVoltage(
                turnFeedback.calculate(inputs.turnPosition.getRadians(), angleSetpoint.getRadians())
            );

            if (speedSetpoint != null) {
                double adjustSpeedSetpoint = speedSetpoint * Math.cos(turnFeedback.getError());

                double velocityRadPerSec = adjustSpeedSetpoint / WHEEL_RADIUS;
                io.setDriveVoltage(
                    driveFeedForward.calculate(velocityRadPerSec)
                    + driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec)
                );
            }
        }
    }

    public SwerveModuleState runSetpoint(SwerveModuleState state) {
        state.optimize(getAngle());

        angleSetpoint = state.angle;
        speedSetpoint = state.speedMetersPerSecond;

        return state;
    }

    public void runCharacterization(double volts) {
        angleSetpoint = new Rotation2d();

        io.setDriveVoltage(volts);
        speedSetpoint = null;
    }

    public void stop() {
        io.setTurnVoltage(0.0);
        io.setDriveVoltage(0.0);

        angleSetpoint = null;
        speedSetpoint = null;
    }

    public void setBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
        io.setTurnBrakeMode(enabled);
    }

    public Rotation2d getAngle() {
        if (turnRelativeOffset == null) {
            return new Rotation2d();
        } else {
            return inputs.turnPosition.plus(turnRelativeOffset);
        }
    }

    public double getPositionMeters() {
        return inputs.drivePositionRad * WHEEL_RADIUS;
    }

    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * WHEEL_RADIUS;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    public double getCharacterizationVelocity() {
        return inputs.driveVelocityRadPerSec;
    }
}
