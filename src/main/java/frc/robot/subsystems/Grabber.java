package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperstructureConstants;

public class Grabber extends SubsystemBase {

    private final SparkMax jointMotor;
    private final SparkMax intakeMotor;
    private final CANcoder jointEncoder;
    private final PIDController jointPID;
    
    private double targetAngle = 0.0;
    private static final double MIN_ANGLE = -90.0;
    private static final double MAX_ANGLE = 90.0;
    

    private static final double DEFAULT_INTAKE_SPEED = 0.7;
    private static final double DEFAULT_OUTTAKE_SPEED = -0.7;

    public Grabber() {
        jointMotor = new SparkMax(SuperstructureConstants.GRABBER_JOINT_MOTOR_ID, MotorType.kBrushless);
        jointMotor.configure(SuperstructureConstants.GRABBER_JOINT_MOTOR_CONFIGURATION, 
                           ResetMode.kResetSafeParameters,
                           PersistMode.kNoPersistParameters);
        
        intakeMotor = new SparkMax(SuperstructureConstants.GRABBER_MOTOR_ID, MotorType.kBrushless);
        intakeMotor.configure(SuperstructureConstants.GRABBER_MOTOR_CONFIGURATION, 
                            ResetMode.kResetSafeParameters,
                            PersistMode.kNoPersistParameters);
        
        jointEncoder = new CANcoder(SuperstructureConstants.GRABBER_CANCODER_ID);

        
        jointPID = new PIDController(0.1, 0, 0);
        jointPID.enableContinuousInput(-180, 180);
        

        SmartDashboard.putNumber("Joint Target Angle", 0.0);
        SmartDashboard.putBoolean("Reset Joint Angle", false);
        

        SmartDashboard.putNumber("Intake Speed", DEFAULT_INTAKE_SPEED);
        SmartDashboard.putNumber("Outtake Speed", DEFAULT_OUTTAKE_SPEED);
        SmartDashboard.putBoolean("Intake", false);
        SmartDashboard.putBoolean("Outtake", false);
    }

    public void setSpeed(double intakeMotorSpeed) {
        intakeMotor.set(intakeMotorSpeed);
    }
    

    public void intake() {
        double intakeSpeed = SmartDashboard.getNumber("Intake Speed", DEFAULT_INTAKE_SPEED);
        setSpeed(intakeSpeed);
    }
    
    public void outtake() {
        double outtakeSpeed = SmartDashboard.getNumber("Outtake Speed", DEFAULT_OUTTAKE_SPEED);
        setSpeed(outtakeSpeed);
    }
    
    public void stopIntake() {
        setSpeed(0);
    }
    
    public double getJointAngle() {
        return jointEncoder.getAbsolutePosition().getValueAsDouble();
    }
    
    public void setJointAngle(double angle) {
        targetAngle = MathUtil.clamp(angle, MIN_ANGLE, MAX_ANGLE);
    }
    
    public void resetJointAngle() {
        jointEncoder.setPosition(0);
        targetAngle = 0;
    }
    
    @Override
    public void periodic() {

        double dashboardAngle = SmartDashboard.getNumber("Joint Target Angle", 0.0);
        setJointAngle(dashboardAngle);
        

        boolean resetRequested = SmartDashboard.getBoolean("Reset Joint Angle", false);
        if (resetRequested) {
            resetJointAngle();
            SmartDashboard.putBoolean("Reset Joint Angle", false);
        }
        

        boolean intakeRequested = SmartDashboard.getBoolean("Intake", false);
        boolean outtakeRequested = SmartDashboard.getBoolean("Outtake", false);
        
        if (intakeRequested) {
            intake();
        } else if (outtakeRequested) {
            outtake();
        } else {
            stopIntake();
        }
        

        double currentAngle = getJointAngle();
        double output = jointPID.calculate(currentAngle, targetAngle);
        output = MathUtil.clamp(output, -1.0, 1.0);
        jointMotor.set(output);
        

        SmartDashboard.putNumber("Current Joint Angle", currentAngle);
        SmartDashboard.putNumber("Joint Motor Output", output);
        SmartDashboard.putNumber("Intake Motor Speed", intakeMotor.get());
    }
}