package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperstructureConstants;

public class Grabber extends SubsystemBase {

    private final SparkMax motor;

    private static Grabber mInstance = null;

    public static Grabber getInstance() {
        if (mInstance == null) {
            mInstance = new Grabber();
        }
        return mInstance;
    }

    public Grabber() {
        motor = new SparkMax(SuperstructureConstants.GRABBER_MOTOR_ID, MotorType.kBrushless);
        motor.configure(SuperstructureConstants.GRABBER_MOTOR_CONFIGURATION, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("grabber set speed", motor.get());
        SmartDashboard.putNumber("grabber rpm", motor.getEncoder().getVelocity());
    }
}
