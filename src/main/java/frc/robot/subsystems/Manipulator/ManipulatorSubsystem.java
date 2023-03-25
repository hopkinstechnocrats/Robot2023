package frc.robot.subsystems.Manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;


public class ManipulatorSubsystem extends SubsystemBase {
    SparkMaxPIDController manipulatorPID;
    CANSparkMax manipulatorMotor;
    RelativeEncoder manipulatorEncoder;
    NetworkTableInstance inst;
    NetworkTable table;
    NetworkTableEntry setpointLog;
    NetworkTableEntry currentVelLog;
    double setDirection = 0;
    
    public ManipulatorSubsystem () {
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Manipulator");
        setpointLog = table.getEntry("Setpoint (RPM)");
        currentVelLog = table.getEntry("Current Velocity (RPM)");
        manipulatorMotor = new CANSparkMax(ManipulatorConstants.kMotorPort, MotorType.kBrushless);
        manipulatorEncoder = manipulatorMotor.getEncoder();
        manipulatorEncoder.setVelocityConversionFactor(ManipulatorConstants.kGearRatio);
        manipulatorMotor.setSmartCurrentLimit(15, 30);

        manipulatorMotor.burnFlash(); // Save settings even after brownout

        // manipulatorPID = manipulatorMotor.getPIDController(); PID
        // manipulatorPID.setFeedbackDevice(manipulatorEncoder);
        // manipulatorPID.setP(ManipulatorConstants.kP);
        // manipulatorPID.setI(ManipulatorConstants.kI);
        // manipulatorPID.setD(ManipulatorConstants.kD);
        // manipulatorPID.setFF(ManipulatorConstants.kF);
        // manipulatorPID.setOutputRange(-ManipulatorConstants.kMaxSpeedRPM, ManipulatorConstants.kMaxSpeedRPM);

    }

    public void Spin() {
            Spin(setDirection * ManipulatorConstants.kCubeSpeedIn);
    }

    public void spinConeOut() {
        setDirection = -1;
    }

    public void spinConeIn() {
        setDirection = 1;
    }

    public void stopSpin() {
        setDirection = 0;
    }

    public void SpinCone(Boolean direction) {
        if (direction) {
            Spin(ManipulatorConstants.kConeSpeedIn);
        } else {
            Spin(ManipulatorConstants.kConeSpeedOut);
        }
    }
    

    public void NoSpin() {
        // manipulatorPID.setReference(0, ControlType.kVelocity); PID
        manipulatorMotor.set(0);
    }

    public void Spin(double setpoint) {
        // manipulatorPID.setReference(setpoint, ControlType.kVelocity); PID
        // setpointLog.setDouble(setpoint);
        // currentVelLog.setDouble(manipulatorEncoder.getVelocity());
        manipulatorMotor.set(setpoint);
        
    }
}
