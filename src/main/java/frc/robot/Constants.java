package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import frc.FSLib2025.swerve.SwerveModuleConstants;

public class Constants {

    public static final class RobotConstants {
        public static final double PERIODIC_INTERVAL = 0.02; // the periodic ,in seconds
        public static final String CANBUS_NAME = "GTX7130";
        public static final int DRIVE_CONTROLLER_PORT = 0;
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
        // elevator motor constants
        public static final int LEFT_ELEVATOR_MOTOR_ID = 0;
        public static final int RIGHT_ELEVATOR_MOTOR_ID = 0;

        // elevator cancoder constants
        public static final int ELEVATOR_CANCODER_ID = 0;

        // elevator motor configuration
        public static final SparkMaxConfig LEFT_ELEVATOR_MOTOR_CONFIGURATION = new SparkMaxConfig();
        static {
            LEFT_ELEVATOR_MOTOR_CONFIGURATION.inverted(false);
        }

        public static final SparkMaxConfig RIGHT_ELEVATOR_MOTOR_CONFIGURATION = new SparkMaxConfig();
        static {
            RIGHT_ELEVATOR_MOTOR_CONFIGURATION.inverted(false);
        }

        // elevator cancoder configuration
        public static final CANcoderConfiguration ELEVATOR_CANCODER_CONFIGURATION = new CANcoderConfiguration();
        static {
            ELEVATOR_CANCODER_CONFIGURATION.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
            ELEVATOR_CANCODER_CONFIGURATION.MagnetSensor.MagnetOffset = 0;
            ELEVATOR_CANCODER_CONFIGURATION.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        }

        // arm (joint) motor constants
        public static final int ARM_MOTOR_ID = 0;

        // arm motor configuration
        public static final SparkMaxConfig ARM_MOTOR_CONFIGURATION = new SparkMaxConfig();
        static {
        }

        // intaker motor constants
        public static final int INTAKER_MOTOR_ID = 0;

        // intaker motor configuration
        public static final SparkMaxConfig INTAKER_MOTOR_CONFIGURATION = new SparkMaxConfig();
        static {
            INTAKER_MOTOR_CONFIGURATION.inverted(false);
        }
        
        
        // grabber
        public static final int GRABBER_UPPER_MOTOR_ID = 2;

        // grabber upper motor configuration
        public static final SparkMaxConfig GRABBER_UPPER_MOTOR = new SparkMaxConfig();
        static {
            GRABBER_UPPER_MOTOR.inverted(false);
        }
        public static final int GRABBER_LOWER_MOTOR_ID = 1;

        // grabber lower motor configuration
        public static final SparkMaxConfig GRABBER_LOWER_MOTOR = new SparkMaxConfig();
        static {
                GRABBER_LOWER_MOTOR.inverted(false);
        }
    }
}
