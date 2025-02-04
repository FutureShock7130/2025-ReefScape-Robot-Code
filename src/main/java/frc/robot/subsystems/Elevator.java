package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperstructureConstants;

public class Elevator extends SubsystemBase {
    // motors
    private final SparkMax leftElevatorMotor;
    private final SparkMax rightElevatorMotor;

    // cancoder
    private final CANcoder cancoder;

    public Elevator() {
        leftElevatorMotor = new SparkMax(SuperstructureConstants.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        leftElevatorMotor.configure(SuperstructureConstants.LEFT_ELEVATOR_MOTOR_CONFIGURATION,
                ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        rightElevatorMotor = new SparkMax(SuperstructureConstants.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        rightElevatorMotor.configure(SuperstructureConstants.RIGHT_ELEVATOR_MOTOR_CONFIGURATION,
                ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        cancoder = new CANcoder(SuperstructureConstants.ELEVATOR_CANCODER_ID);
        cancoder.getConfigurator().apply(SuperstructureConstants.ELEVATOR_CANCODER_CONFIGURATION);
    }

    public void setElevatorVoltage(double voltage) {
        leftElevatorMotor.setVoltage(voltage);
        rightElevatorMotor.setVoltage(voltage);
    }

    private Distance getElevatorHeight() {
        return Meters.of(getElevatorMotorPosition().in(Rotations) * 2 * Math.PI);
    }

    private Angle getElevatorMotorPosition() {
        return Rotations
                .of((leftElevatorMotor.getEncoder().getPosition() + rightElevatorMotor.getEncoder().getPosition()) / 2);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elevator position", getElevatorHeight().in(Meters));
    }
}
