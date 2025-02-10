// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.IO.ModuleIO;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class Swerve implements ModuleIO {

    private static Swerve m_instance = null;

    public static Swerve getInstance(int index) {
        if (m_instance == null) {
            m_instance = new Swerve(index);
        }
        return m_instance;
    }

    private final TalonFX driveTalon;
    // private final TalonFX turnTalon;
    private final SparkMax turnSparkMax;
    private final CANcoder cancoder;

    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveCurrent;

    private final StatusSignal<Angle> turnAbsolutePosition;

    private final Rotation2d absoluteEncoderOffset;

    public Swerve(int index) {
        if (index < 0 || index >= SwerveConstants.SWERVE_MODULE_CONSTANTS.length) {
            throw new IllegalArgumentException("Invalid index: " + index);
        }

        driveTalon = new TalonFX(SwerveConstants.SWERVE_MODULE_CONSTANTS[index].DriveMotorId, RobotConstants.CANBUS_NAME);
        // turnTalon = new TalonFX(SwerveConstants.SWERVE_MODULE_CONSTANTS[index].SteerMotorId, RobotConstants.CANBUS_NAME);
        turnSparkMax = new SparkMax(SwerveConstants.SWERVE_MODULE_CONSTANTS[index].SteerMotorId, MotorType.kBrushless);
        cancoder = new CANcoder(SwerveConstants.SWERVE_MODULE_CONSTANTS[index].CANcoderId);
        absoluteEncoderOffset = new Rotation2d(Units.rotationsToRadians(SwerveConstants.SWERVE_MODULE_CONSTANTS[index].CANcoderOffset));

        TalonFXConfiguration driveTalonConfig = SwerveConstants.DRIVE_MOTOR_CONFIGURATION;
        driveTalon.getConfigurator().apply(driveTalonConfig);
        setDriveMotorOutputMode(true);

        SparkMaxConfig turnSparkConfig = SwerveConstants.STEER_MOTOR_CONFIGURATION;
        turnSparkMax.setCANTimeout(250);
        turnSparkMax.configure(turnSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        cancoder.getConfigurator().apply(new CANcoderConfiguration());

        drivePosition = driveTalon.getPosition();
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveCurrent = driveTalon.getSupplyCurrent();

        turnAbsolutePosition = cancoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(100.0, drivePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            driveVelocity,
            driveAppliedVolts,
            driveCurrent,
            turnAbsolutePosition
            //     turnVelocity,
            //     turnAppliedVolts,
            //     turnCurrent
            );
        driveTalon.optimizeBusUtilization();
        // turnTalon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            drivePosition,
            driveVelocity,
            driveAppliedVolts,
            driveCurrent,
            turnAbsolutePosition
        //     turnVelocity,
        //     turnAppliedVolts,
        //     turnCurrent
        );

        inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble() / SwerveConstants.DRIVE_MOTOR_GEAR_RATIO);
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble() / SwerveConstants.DRIVE_MOTOR_GEAR_RATIO);
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

        inputs.turnAbsolutePosition =
            new Rotation2d(Units.rotationsToRadians(cancoder.getAbsolutePosition().getValueAsDouble())).plus(absoluteEncoderOffset);
        inputs.turnPosition =
            Rotation2d.fromRotations(turnSparkMax.getEncoder().getPosition() / SwerveConstants.STEER_MOTOR_GEAR_RATIO);
        inputs.turnVelocityRadPerSec =
            Units.rotationsPerMinuteToRadiansPerSecond(turnSparkMax.getEncoder().getVelocity()) / SwerveConstants.STEER_MOTOR_GEAR_RATIO;
        inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
        inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
        // inputs.turnPosition =
        // Rotation2d.fromRotations(turnPosition.getValueAsDouble() / TURN_GEAR_RATIO);
        // inputs.turnVelocityRadPerSec =
        //     Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / TURN_GEAR_RATIO;
        // inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
        // inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveTalon.setControl(new VoltageOut(volts));
    }

    @Override
    public void setTurnVoltage(double volts) {
        // turnTalon.setControl(new VoltageOut(volts));
        turnSparkMax.setVoltage(volts);
    }

    public void setDriveMotorOutputMode(boolean enable) {
        var config = new MotorOutputConfigs();
        config.Inverted = InvertedValue.CounterClockwise_Positive;
        config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveTalon.getConfigurator().apply(config);
    }

    // public void setTurnMotorOutputMode(boolean enable) {
    //     var config = new MotorOutputConfigs();
    //     config.Inverted = 
    //         SwerveConstants.IS_STEER_MOTOR_INVERTED
    //             ? InvertedValue.Clockwise_Positive
    //             : InvertedValue.CounterClockwise_Positive;
    //     config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    //     turnTalon.getConfigurator().apply(config);
    // }
}
