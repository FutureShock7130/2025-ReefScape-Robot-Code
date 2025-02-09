// put here as a tribute to the old swerve drive code programmed by CHenry
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.FSLib2025.math.Maths;
import frc.FSLib2025.swerve.OnboardModuleState;
import frc.FSLib2025.swerve.SwerveModuleConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModuleOld {
    public int ModuleNumber;
    public SwerveModuleConstants ModuleConstants;

    private SparkMax steerMotor;
    private TalonFX driveMotor;

    private CANcoder steerCANcoder;

    private PIDController steerPID;

    private Rotation2d lastAngle;

    public SwerveModuleOld(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.ModuleNumber = moduleNumber;
        this.ModuleConstants = moduleConstants;

        steerPID = new PIDController(
                SwerveConstants.STEER_MOTOR_KP,
                SwerveConstants.STEER_MOTOR_KI,
                SwerveConstants.STEER_MOTOR_KD);

        lastAngle = new Rotation2d();

        steerMotor = new SparkMax(moduleConstants.SteerMotorId, MotorType.kBrushless);
        steerMotor.configure(SwerveConstants.STEER_MOTOR_CONFIGURATION, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        driveMotor = new TalonFX(moduleConstants.DriveMotorId, RobotConstants.CANBUS_NAME);
        driveMotor.getConfigurator().apply(SwerveConstants.DRIVE_MOTOR_CONFIGURATION);
        driveMotor.setNeutralMode(NeutralModeValue.Brake);

        steerCANcoder = new CANcoder(moduleConstants.CANcoderId, RobotConstants.CANBUS_NAME);
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        cancoderConfig.MagnetSensor.MagnetOffset = moduleConstants.CANcoderOffset;
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        steerCANcoder.getConfigurator().apply(cancoderConfig);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = OnboardModuleState.optimize(desiredState, getModuleState().angle);

        desiredState.angle = Math.abs(desiredState.speedMetersPerSecond) < 0.01 ? lastAngle : desiredState.angle;
        double error = getModuleState().angle.getDegrees() - desiredState.angle.getDegrees();
        error = Maths.constrainAngleDegrees(error);
        double steerOutput = steerPID.calculate(error);
        setSteerMotor(steerOutput);
        lastAngle = desiredState.angle;

        double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.MAX_MODULE_SPEED;
        percentOutput = Maths.clamp(percentOutput, -1, 1);
        setDriveMotor(percentOutput);
    }

    private void setSteerMotor(double speed) {
        steerMotor.set(speed);
    }

    private void setDriveMotor(double speed) {
        driveMotor.set(speed);
    }

    public Rotation2d getSteerAngle() {
        return Rotation2d.fromRotations(steerCANcoder.getAbsolutePosition().getValueAsDouble());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                driveMotor.getPosition().getValueAsDouble() * SwerveConstants.DRIVE_WHEEL_PERIMETER,
                getSteerAngle());
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
                driveMotor.getVelocity().getValueAsDouble() * SwerveConstants.DRIVE_WHEEL_PERIMETER,
                getSteerAngle());
    }

}