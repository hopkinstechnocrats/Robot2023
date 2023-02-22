package frc.robot.subsystems.Manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
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
    
    public void ManipulatorSubsystem () {
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Manipulator");
        setpointLog = table.getEntry("Setpoint (RPM)");
        currentVelLog = table.getEntry("Current Velocity (RPM)");
        manipulatorMotor = new CANSparkMax(ManipulatorConstants.kMotorPort, MotorType.kBrushless);
        manipulatorEncoder = manipulatorMotor.getEncoder();
        manipulatorEncoder.setVelocityConversionFactor(ManipulatorConstants.kGearRatio);

        manipulatorPID = manipulatorMotor.getPIDController();
        manipulatorPID.setFeedbackDevice(manipulatorEncoder);
        manipulatorPID.setP(ManipulatorConstants.kP);
        manipulatorPID.setI(ManipulatorConstants.kI);
        manipulatorPID.setD(ManipulatorConstants.kD);
        manipulatorPID.setFF(ManipulatorConstants.kF);
        manipulatorPID.setOutputRange(-ManipulatorConstants.kMaxSpeedRPM, ManipulatorConstants.kMaxSpeedRPM);

    }

    public void SpinCube(Boolean direction) {
        if (direction) {
            Spin(ManipulatorConstants.kCubeSpeedIn);
        } else {
            Spin(ManipulatorConstants.kCubeSpeedOut);
        }
    }

    public void SpinCone(Boolean direction) {
        if (direction) {
            Spin(ManipulatorConstants.kConeSpeedIn);
        } else {
            Spin(ManipulatorConstants.kConeSpeedOut);
        }
    }
    

    public void NoSpin() {
        manipulatorPID.setReference(0, ControlType.kVelocity);
    }

    public void Spin(double setpoint) {
        manipulatorPID.setReference(setpoint, ControlType.kVelocity);
        setpointLog.setDouble(setpoint);
        currentVelLog.setDouble(manipulatorEncoder.getVelocity());
        
    }
}
