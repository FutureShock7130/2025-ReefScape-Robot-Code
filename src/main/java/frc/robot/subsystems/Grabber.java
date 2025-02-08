package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FSLib2025.state_machine.SuperstructureState;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SuperstructureConstants;

public class Grabber extends SubsystemBase {
    /*
     * C shape stuff with 2 sets of 2 jelly wheels at the end of both side
     * locked in elevator's carriage
     * change its angle to place and intake CORAL
     * 
     * control jelly wheels by NEO 550
     * control its angle by NEO Brushless (gear ratio : 18.75)
     */

    // motors for jelly wheels
    private final SparkMax upMotor;
    private final SparkMax lowMotor;

    // motors for angle
    private final SparkMax angleMotor;

    // cancoder
    private final CANcoder cancoder;

    // shuffleboard
    ShuffleboardTab tab = Shuffleboard.getTab("Grabber");
    GenericEntry upMotorSpeed = tab.add("Up Motor Speed (RPM)", 0)
            .withPosition(0, 0).withSize(2, 1)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
    GenericEntry lowMotorSpeed = tab.add("Low Motor Speed (RPM)", 0)
            .withPosition(1, 0).withSize(2, 1)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
    GenericEntry angle = tab.add("Grabber Angle (degrees)", 0)
            .withPosition(2, 0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

    public Grabber() {
        upMotor = new SparkMax(SuperstructureConstants.GRABBER_UP_MOTOR_ID, MotorType.kBrushless);
        upMotor.configure(SuperstructureConstants.GRABBER_UP_MOTOR_CONFIGURATION, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        lowMotor = new SparkMax(SuperstructureConstants.GRABBER_LOW_MOTOR_ID, MotorType.kBrushless);
        lowMotor.configure(SuperstructureConstants.GRABBER_LOW_MOTOR_CONFIGURATION, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        angleMotor = new SparkMax(SuperstructureConstants.GRABBER_ANGLE_MOTOR_ID, MotorType.kBrushless);
        angleMotor.configure(SuperstructureConstants.GRABBER_ANGLE_MOTOR_CONFIGURATION, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);

        cancoder = new CANcoder(SuperstructureConstants.GRABBER_CANCODER_ID, RobotConstants.CANBUS_NAME);
        cancoder.getConfigurator().apply(SuperstructureConstants.GRABBER_CANCODER_CONFIGURATION);
    }

    public void setUpMotorVolt(double volts) {
        upMotor.setVoltage(volts);
    }

    public void setUpMotorSpeed(double speed) {
        upMotor.set(speed);
    }

    public void setLowMotorVolt(double volts) {
        lowMotor.setVoltage(volts);
    }

    public void setLowMotorSpeed(double speed) {
        lowMotor.set(speed);
    }

    public void setAngleMotorVolt(double volts) {
        angleMotor.setVoltage(volts);
    }

    public void setAngleMotorSpeed(double speed) {
        angleMotor.set(speed);
    }

    public double getUpMotorSpeed() {
        return upMotor.getEncoder().getVelocity();
    }

    public double getLowMotorSpeed() {
        return lowMotor.getEncoder().getVelocity();
    }

    public double getAngle() {
        return Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble()).getDegrees();
    }

    @Override
    public void periodic() {
        upMotorSpeed.setDouble(getUpMotorSpeed());
        lowMotorSpeed.setDouble(getLowMotorSpeed());
        angle.setDouble(getAngle());
    }
}
