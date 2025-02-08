package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperstructureConstants;

public class Elevator extends SubsystemBase {
    /*
     * 3 stage elevator and 1 carriage
     */

    // motors
    private final SparkMax leftElevatorMotor;
    private final SparkMax rightElevatorMotor;

    // cancoder
    private final CANcoder cancoder;

    private static Elevator m_Instance = null;
    
    public static Elevator getInstance() {
        if (m_Instance == null) {
            m_Instance = new Elevator();
        }
        return m_Instance;
    }

    // shuffleboard
    ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
    GenericEntry elevatorHeight = tab.add("Elevator Height", 0)
        .withPosition(0, 0).withSize(2, 1)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

    public Elevator() {
        leftElevatorMotor = new SparkMax(SuperstructureConstants.ELEVATOR_LEFT_MOTOR_ID, MotorType.kBrushless);
        leftElevatorMotor.configure(SuperstructureConstants.ELEVATOR_LEFT_MOTOR_CONFIGURATION,
                ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        rightElevatorMotor = new SparkMax(SuperstructureConstants.ELEVATOR_RIGHT_MOTOR_ID, MotorType.kBrushless);
        rightElevatorMotor.configure(SuperstructureConstants.ELEVATOR_RIGHT_MOTOR_CONFIGURATION,
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
        elevatorHeight.setDouble(getElevatorHeight().in(Meters));
    }
}
