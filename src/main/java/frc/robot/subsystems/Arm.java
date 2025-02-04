package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperstructureConstants;

public class Arm extends SubsystemBase {

    private final SparkMax motor;

    public Arm() {
        motor = new SparkMax(SuperstructureConstants.ARM_MOTOR_ID, MotorType.kBrushless);
        motor.configure(SuperstructureConstants.ARM_MOTOR_CONFIGURATION, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public Rotation2d getArmAngle() {
        return Rotation2d.fromRotations(motor.getEncoder().getPosition());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("arm position (rot)", getArmAngle().getRotations());
    }
}
