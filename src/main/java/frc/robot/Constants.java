package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import frc.FSLib2025.swerve.SwerveModuleConstants;

public class Constants {

    public static final class RobotConstants {
        public static final double PERIODIC_INTERVAL = 0.02; // the periodic ,in seconds
        public static final String CANBUS_NAME = "GTX7130";
        public static final int DRIVE_CONTROLLER_PORT = 0;
        public static final int OPERATOR_BUTTONBOX_PORT = 1;
    }

    public static final class SwerveConstants {
        public static final double MAX_MODULE_SPEED = 4;
        public static final double MAX_MODULE_ROTATIONAL_SPEED = 4;
        public static final double WHEEL_BASE = 0.583;

        public static final int PIGEON_ID = 40;

        public static final double STEER_MOTOR_KP = 0.008;
        public static final double STEER_MOTOR_KI = 0.05;
        public static final double STEER_MOTOR_KD = 0.005;
        public static final double STEER_MOTOR_WINDUP = 0.0;
        public static final int STEER_MOTOR_LIMIT = 0;

        public static final double DRIVE_MOTOR_GEAR_RATIO = 6.122449;
        public static final double DRIVE_WHEEL_DIAMETERS = 4 * 0.0254; // meters
        public static final double DRIVE_WHEEL_PERIMETER = Math.PI * DRIVE_WHEEL_DIAMETERS; // meters

        public static final Translation2d[] MODULE_TRANSLATOIN_METERS = new Translation2d[] {
                new Translation2d(WHEEL_BASE / 2.0, WHEEL_BASE / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -WHEEL_BASE / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -WHEEL_BASE / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, WHEEL_BASE / 2.0)
        };

        public static final SwerveModuleConstants MOD0_CONSTANTS = new SwerveModuleConstants();
        static {
            MOD0_CONSTANTS.DriveMotorId = 1;
            MOD0_CONSTANTS.SteerMotorId = 2;
            MOD0_CONSTANTS.CANcoderId = 0;
            MOD0_CONSTANTS.CANcoderOffset = -0.272461;
        }

        public static final SwerveModuleConstants MOD1_CONSTANTS = new SwerveModuleConstants();
        static {
            MOD1_CONSTANTS.DriveMotorId = 11;
            MOD1_CONSTANTS.SteerMotorId = 12;
            MOD1_CONSTANTS.CANcoderId = 1;
            MOD1_CONSTANTS.CANcoderOffset = -0.123047;
        }

        public static final SwerveModuleConstants MOD2_CONSTANTS = new SwerveModuleConstants();
        static {
            MOD2_CONSTANTS.DriveMotorId = 21;
            MOD2_CONSTANTS.SteerMotorId = 22;
            MOD2_CONSTANTS.CANcoderId = 2;
            MOD2_CONSTANTS.CANcoderOffset = 0.205322;
        }

        public static final SwerveModuleConstants MOD3_CONSTANTS = new SwerveModuleConstants();
        static {
            MOD3_CONSTANTS.DriveMotorId = 31;
            MOD3_CONSTANTS.SteerMotorId = 32;
            MOD3_CONSTANTS.CANcoderId = 3;
            MOD3_CONSTANTS.CANcoderOffset = -0.119141;
        }

        // motor configuration
        public static final TalonFXConfiguration DRIVE_MOTOR_CONFIGURATION = new TalonFXConfiguration();
        static {
            DRIVE_MOTOR_CONFIGURATION.Feedback.SensorToMechanismRatio = SwerveConstants.DRIVE_MOTOR_GEAR_RATIO;
        }

        public static final SparkMaxConfig STEER_MOTOR_CONFIGURATION = new SparkMaxConfig();
        static {
            STEER_MOTOR_CONFIGURATION.inverted(false);
            STEER_MOTOR_CONFIGURATION.idleMode(IdleMode.kBrake);
        }
    }

    public static final class SuperstructureConstants {
        /*************** ELEVATOR CONSTANTS ***************/
        // elevator motor constants
        // TODO: fill the int
        public static final int LEFT_ELEVATOR_MOTOR_ID = 0;
        public static final int RIGHT_ELEVATOR_MOTOR_ID = 0;
        public static final int ELEVATOR_LEFT_MOTOR_ID = 0;
        public static final int ELEVATOR_RIGHT_MOTOR_ID = 0;

        // elevator cancoder constants
        // TODO: fill the int
        public static final int ELEVATOR_CANCODER_ID = 0;
        public static final SparkMaxConfig LEFT_ELEVATOR_MOTOR_CONFIGURATION = new SparkMaxConfig();

        // elevator motor configuration
        public static final SparkMaxConfig ELEVATOR_LEFT_MOTOR_CONFIGURATION = new SparkMaxConfig();
        static {
            ELEVATOR_LEFT_MOTOR_CONFIGURATION.inverted(false);
            ELEVATOR_LEFT_MOTOR_CONFIGURATION.idleMode(IdleMode.kBrake);
            ELEVATOR_LEFT_MOTOR_CONFIGURATION.voltageCompensation(12);
            ELEVATOR_LEFT_MOTOR_CONFIGURATION.smartCurrentLimit(40);
        }

        public static final SparkMaxConfig ELEVATOR_RIGHT_MOTOR_CONFIGURATION = new SparkMaxConfig();
        static {
            ELEVATOR_RIGHT_MOTOR_CONFIGURATION.inverted(false);
            ELEVATOR_RIGHT_MOTOR_CONFIGURATION.idleMode(IdleMode.kBrake);
            ELEVATOR_RIGHT_MOTOR_CONFIGURATION.voltageCompensation(12);
            ELEVATOR_RIGHT_MOTOR_CONFIGURATION.smartCurrentLimit(40);
        }

        // elevator cancoder configuration
        public static final CANcoderConfiguration ELEVATOR_CANCODER_CONFIGURATION = new CANcoderConfiguration();
        static {
            ELEVATOR_CANCODER_CONFIGURATION.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
            ELEVATOR_CANCODER_CONFIGURATION.MagnetSensor.MagnetOffset = 0;
            ELEVATOR_CANCODER_CONFIGURATION.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        }

        // elevator pid
        public static final double ELEVATOR_KP = 0;
        public static final double ELEVATOR_KI = 0;
        public static final double ELEVATOR_KD = 0;

        // elevator feedforward
        public static final double ELEVATOR_KS = 0.03;

        /*************** GRABBER CONSTANTS ***************/
        // grabber motor constants
        public static final int GRABBER_UP_MOTOR_ID = 0;
        public static final int GRABBER_LOW_MOTOR_ID = 0;
        public static final int GRABBER_ANGLE_MOTOR_ID = 0;
        
        // grabber cancoder constants
        public static final int GRABBER_CANCODER_ID = 0;

        // grabber ratio
        public static final double GRABBER_ANGLE_RATIO = 18.75; // 3:1, 5:1, 12->15

        // grabber motor configuration
        public static final SparkMaxConfig GRABBER_UP_MOTOR_CONFIGURATION = new SparkMaxConfig();
        static {
            GRABBER_UP_MOTOR_CONFIGURATION.inverted(true);
            GRABBER_UP_MOTOR_CONFIGURATION.idleMode(IdleMode.kBrake);
            GRABBER_UP_MOTOR_CONFIGURATION.voltageCompensation(12);
        }
        public static final SparkMaxConfig GRABBER_LOW_MOTOR_CONFIGURATION = new SparkMaxConfig();
        static {
            GRABBER_LOW_MOTOR_CONFIGURATION.inverted(false);
            GRABBER_LOW_MOTOR_CONFIGURATION.idleMode(IdleMode.kBrake);
            GRABBER_LOW_MOTOR_CONFIGURATION.voltageCompensation(12);
        }
        public static final SparkMaxConfig GRABBER_ANGLE_MOTOR_CONFIGURATION = new SparkMaxConfig();
        static {
            GRABBER_ANGLE_MOTOR_CONFIGURATION.inverted(false);
            GRABBER_ANGLE_MOTOR_CONFIGURATION.idleMode(IdleMode.kBrake);
            GRABBER_ANGLE_MOTOR_CONFIGURATION.voltageCompensation(12);
            GRABBER_ANGLE_MOTOR_CONFIGURATION.smartCurrentLimit(50);
            GRABBER_ANGLE_MOTOR_CONFIGURATION.absoluteEncoder.positionConversionFactor(GRABBER_ANGLE_RATIO);
        }

        // grabber cancoder configuration
        public static final CANcoderConfiguration GRABBER_CANCODER_CONFIGURATION = new CANcoderConfiguration();
        static {
            GRABBER_CANCODER_CONFIGURATION.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
            GRABBER_CANCODER_CONFIGURATION.MagnetSensor.MagnetOffset = 0;
            GRABBER_CANCODER_CONFIGURATION.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        }

        // grabber pid
        public static final double GRABBER_KP = 0;
        public static final double GRABBER_KI = 0;
        public static final double GRABBER_KD = 0;

        // grabber feedforward
        public static final double GRABBER_KS = 0;
        public static final double GRABBER_KV = 0;
        public static final double GRABBER_KA = 0;

        /*************** INTAKE CONSTANTS ***************/
        // intake motor constants
        public static final int INTAKE_MOTOR_ID = 0;
        public static final int INTAKE_ANGLE_MOTOR_ID = 0;

        // intake cancoder constants
        public static final int INTAKE_CANCODER_ID = 0;

        // intake ratio
        public static final double INTAKE_ANGLE_RATIO = 61.875; // 5:1, 9:1, 16->22

        // intake motor configuration
        public static final SparkMaxConfig INTAKE_MOTOR_CONFIGURATION = new SparkMaxConfig();
        static {
            INTAKE_MOTOR_CONFIGURATION.inverted(false);
            INTAKE_MOTOR_CONFIGURATION.idleMode(IdleMode.kCoast);
            INTAKE_MOTOR_CONFIGURATION.voltageCompensation(12);
        }
        public static final SparkMaxConfig INTAKE_ANGLE_MOTOR_CONFIGURATION = new SparkMaxConfig();
        static {
            INTAKE_ANGLE_MOTOR_CONFIGURATION.inverted(false);
            INTAKE_ANGLE_MOTOR_CONFIGURATION.idleMode(IdleMode.kBrake);
            INTAKE_ANGLE_MOTOR_CONFIGURATION.voltageCompensation(12);
        }

        // intake cancoder configuration
        public static final CANcoderConfiguration INTAKE_CANCODER_CONFIGURATION = new CANcoderConfiguration();
        static {
            INTAKE_CANCODER_CONFIGURATION.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
            INTAKE_CANCODER_CONFIGURATION.MagnetSensor.MagnetOffset = 0;
            INTAKE_CANCODER_CONFIGURATION.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        }

        // intake pid
        public static final double INTAKE_KP = 0;
        public static final double INTAKE_KI = 0;
        public static final double INTAKE_KD = 0;

        /*************** ARM CONSTANTS ***************/
        public static final int ARM_MOTOR_ID = 0;

        public static final SparkMaxConfig ARM_MOTOR_CONFIGURATION = new SparkMaxConfig();
    }

    public static final class JoystickConstants {

    }
}
