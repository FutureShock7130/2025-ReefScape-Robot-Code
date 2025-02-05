package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperstructureConstants;

public class Grabber extends SubsystemBase {
    private final SparkMax upperMotor;
    private final SparkMax lowerMotor;
    private double lastUpperRPM = 0;
    private double lastLowerRPM = 0;
    private static final double STALL_RPM_THRESHOLD = 50.0;
    private static final int STALL_DETECT_COUNT = 3;
    private int stallCount = 0;
    private boolean isStalled = false;
    private double targetUpperSpeed = 0.0;
    private double targetLowerSpeed = 0.0;

    public Grabber() {
        upperMotor = new SparkMax(SuperstructureConstants.GRABBER_UPPER_MOTOR_ID, MotorType.kBrushless);
        lowerMotor = new SparkMax(SuperstructureConstants.GRABBER_LOWER_MOTOR_ID, MotorType.kBrushless);

        upperMotor.configure(SuperstructureConstants.GRABBER_UPPER_MOTOR, 
                           ResetMode.kResetSafeParameters,
                           PersistMode.kNoPersistParameters);
        lowerMotor.configure(SuperstructureConstants.GRABBER_LOWER_MOTOR, 
                           ResetMode.kResetSafeParameters,
                           PersistMode.kNoPersistParameters);
        

        SmartDashboard.putBoolean("Intake", false);
        SmartDashboard.putBoolean("Eject", false);
        SmartDashboard.putBoolean("Stop Grabber", false);

        updateStatusIndicator(false);
    }

    private void updateStatusIndicator(boolean hasGamePiece) {
        SmartDashboard.putString("Grabber Status", hasGamePiece ? "🟢" : "🔴");
    }

    public void setSpeed(double upperSpeed, double lowerSpeed) {
        upperSpeed = Math.max(-1.0, Math.min(1.0, upperSpeed));
        lowerSpeed = Math.max(-1.0, Math.min(1.0, lowerSpeed));
        
        targetUpperSpeed = upperSpeed;
        targetLowerSpeed = lowerSpeed;
        
        upperMotor.set(upperSpeed);
        lowerMotor.set(lowerSpeed);
    }

    public void intake() {
        setSpeed(0.7, 0.7);
    }

    public void eject() {
        setSpeed(-0.7, -0.7);
    }

    public boolean hasGamepiece() {
        if (Math.abs(upperMotor.get()) < 0.1 || Math.abs(lowerMotor.get()) < 0.1) {
            return false;
        }

        double upperCurrentRPM = Math.abs(upperMotor.getEncoder().getVelocity());
        double lowerCurrentRPM = Math.abs(lowerMotor.getEncoder().getVelocity());

        if (upperCurrentRPM < STALL_RPM_THRESHOLD && 
            lowerCurrentRPM < STALL_RPM_THRESHOLD && 
            Math.abs(upperMotor.get()) > 0.1 && 
            Math.abs(lowerMotor.get()) > 0.1) {
            stallCount++;
        } else {
            stallCount = 0;
        }

        isStalled = stallCount >= STALL_DETECT_COUNT;
        
        lastUpperRPM = upperCurrentRPM;
        lastLowerRPM = lowerCurrentRPM;
        return isStalled;
    }

    public void stop() {
        setSpeed(0, 0);
        stallCount = 0;
        isStalled = false;
    }
    
    @Override
    public void periodic() {
  
        if (SmartDashboard.getBoolean("Intake", false)) {
            intake();
            SmartDashboard.putBoolean("Intake", false);
        }
        else if (SmartDashboard.getBoolean("Eject", false)) {
            eject();
            SmartDashboard.putBoolean("Eject", false);
        }
        else if (SmartDashboard.getBoolean("Stop Grabber", false)) {
            stop();
            SmartDashboard.putBoolean("Stop Grabber", false);
        }

        boolean currentHasGamePiece = hasGamepiece();
        updateStatusIndicator(currentHasGamePiece);

        double upperCurrentRPM = upperMotor.getEncoder().getVelocity();
        double lowerCurrentRPM = lowerMotor.getEncoder().getVelocity();
        double upperCurrent = upperMotor.getOutputCurrent();
        double lowerCurrent = lowerMotor.getOutputCurrent();
        
        SmartDashboard.putNumber("Upper Motor Speed", upperMotor.get());
        SmartDashboard.putNumber("Lower Motor Speed", lowerMotor.get());
        SmartDashboard.putNumber("Upper Motor RPM", upperCurrentRPM);
        SmartDashboard.putNumber("Lower Motor RPM", lowerCurrentRPM);
        SmartDashboard.putNumber("Upper Motor Current", upperCurrent);
        SmartDashboard.putNumber("Lower Motor Current", lowerCurrent);
        SmartDashboard.putBoolean("Is Stalled", isStalled);
        SmartDashboard.putNumber("Stall Count", stallCount);
        SmartDashboard.putBoolean("Has Game Piece", hasGamepiece());
    }
}

