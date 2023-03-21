// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.TestFixtureConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositionConstants;
import frc.robot.Constants.ElevatorConstants.EquationConstants;
import frc.robot.Constants.ElevatorConstants.ExtenderConstants;
import frc.robot.Constants.ElevatorConstants.WinchConstants;
import lib.Loggable;


public class ElevatorSubsystem extends SubsystemBase implements Loggable {
  NetworkTableInstance inst;
  NetworkTable table;
  
  boolean winchZeroSpeedBool = false;
  double winchZeroSpeedDouble = 0;
  boolean extendZeroSpeedBool = false;
  double extendZeroSpeedDouble = 0;

  double winchDesiredSetpoint = 0;
  double extendDesiredSetpoint = 0;
  /** Creates a new SingleModuleTestFixture. */

  private final CANSparkMax m_extendPrimaryMotor = new CANSparkMax(Constants.ElevatorConstants.ExtenderConstants.kPrimaryMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_extendSecondaryMotor = new CANSparkMax(Constants.ElevatorConstants.ExtenderConstants.kSecondaryMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_winchMotor = new CANSparkMax(Constants.ElevatorConstants.WinchConstants.kMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
  //Asumes Hall Effect Encoder
  private final RelativeEncoder m_extendLeftMotorBuiltInEncoder = m_extendPrimaryMotor.getEncoder();
  //private final RelativeEncoder m_extendRightMotorBuiltInEncoder = m_extendRightMotor.getEncoder();
  private final RelativeEncoder m_winchMotorBuiltInEncoder = m_winchMotor.getEncoder();

  private final DutyCycleEncoder m_ThruBoreEncoder = new DutyCycleEncoder(0);

  //Using Left Motor As Primary Motor
  private final SparkMaxPIDController m_extendPIDController = m_extendPrimaryMotor.getPIDController();
  private final SparkMaxPIDController m_winchPIDController = m_winchMotor.getPIDController();

  private final Constraints m_extendContraints = new Constraints(ExtenderConstants.kMaxSpeedRPM, ExtenderConstants.kMaxAccelerationRPMM);
  private final Constraints m_winchContraints = new Constraints(WinchConstants.kMaxSpeedRPM, WinchConstants.kMaxAccelerationRPMM);

  private final ProfiledPIDController m_extendProfiledPIDController = new ProfiledPIDController(ExtenderConstants.kP, ExtenderConstants.kI, ExtenderConstants.kD, m_extendContraints);
  private final ProfiledPIDController m_winchProfiledPIDController = new ProfiledPIDController(WinchConstants.kP, WinchConstants.kI, WinchConstants.kD, m_winchContraints);

  // private final double[][] positionSetpoints;

  public ElevatorSubsystem(){
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("Elevator");

    m_ThruBoreEncoder.setDutyCycleRange(1, 1024);
    
    // m_extendPrimaryMotor.restoreFactoryDefaults();
    // m_extendSecondaryMotor.restoreFactoryDefaults();
    // m_winchMotor.restoreFactoryDefaults();

    // m_extendPrimaryMotor.setIdleMode(IdleMode.kBrake);
    // m_extendSecondaryMotor.setIdleMode(IdleMode.kBrake);
    m_extendSecondaryMotor.follow(m_extendPrimaryMotor, true);

      m_extendPIDController.setFeedbackDevice(m_extendLeftMotorBuiltInEncoder);
      m_winchPIDController.setFeedbackDevice(m_winchMotorBuiltInEncoder);

      m_extendPrimaryMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      m_extendPrimaryMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
      m_winchMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      m_winchMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

      m_extendPrimaryMotor.setSoftLimit(SoftLimitDirection.kForward, 0);
      m_extendPrimaryMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)-16.7);

      m_winchMotor.setSoftLimit(SoftLimitDirection.kForward, 0);
      m_winchMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)-71.4);
  
      m_extendPIDController.setP(Constants.ElevatorConstants.ExtenderConstants.kP);
      m_extendPIDController.setI(Constants.ElevatorConstants.ExtenderConstants.kI);
      m_extendPIDController.setD(Constants.ElevatorConstants.ExtenderConstants.kD);

      m_extendPIDController.setSmartMotionMaxVelocity(ExtenderConstants.kMaxSpeedRPM, 0);
      m_extendPIDController.setSmartMotionMaxAccel(ExtenderConstants.kMaxAccelerationRPMM, 0);
      m_extendPIDController.setSmartMotionMinOutputVelocity(0, 0);
      
      m_extendPIDController.setFF(Constants.ElevatorConstants.ExtenderConstants.kFF);
      // m_extendPIDController.setIAccum(ExtenderConstants.kIAccum);
      // m_extendPIDController.setIZone(Constants.ElevatorConstants.ExtenderConstants.kIz);
      m_extendPIDController.setOutputRange(Constants.ElevatorConstants.ExtenderConstants.kMinOutput, 
      Constants.ElevatorConstants.ExtenderConstants.kMaxOutput);
      // m_extendLeftMotorBuiltInEncoder.setPositionConversionFactor(ExtenderConstants.kMetersPerEncoderTick);
  
      m_winchPIDController.setI(Constants.ElevatorConstants.WinchConstants.kI);
      m_winchPIDController.setP(Constants.ElevatorConstants.WinchConstants.kP);
      m_winchPIDController.setD(Constants.ElevatorConstants.WinchConstants.kD);      

      m_winchPIDController.setSmartMotionMaxVelocity(WinchConstants.kMaxSpeedRPM, 0);
      m_winchPIDController.setSmartMotionMaxAccel(WinchConstants.kMaxAccelerationRPMM, 0);

      m_winchPIDController.setFF(Constants.ElevatorConstants.WinchConstants.kFF);
      // m_winchPIDController.setIZone(Constants.ElevatorConstants.WinchConstants.kIz);
      m_winchPIDController.setOutputRange(Constants.ElevatorConstants.WinchConstants.kMinOutput, Constants.ElevatorConstants.WinchConstants.kMaxOutput);
      // m_winchMotorBuiltInEncoder.setPositionConversionFactor(WinchConstants.kRadiansPerEncoderTick);
      
      // positionSetpoints = new double[][] {
      //   thingsFromCartesianPoint(ElevatorPositionConstants.kPosition1X, ElevatorPositionConstants.kPosition1Y),
      //   thingsFromCartesianPoint(ElevatorPositionConstants.kPositionHomeX, ElevatorPositionConstants.kPositionHomeY),
      //   thingsFromCartesianPoint(ElevatorPositionConstants.kPositionMiddleX, ElevatorPositionConstants.kPositionMiddleY),
      //   thingsFromCartesianPoint(ElevatorPositionConstants.kPositionHighX, ElevatorPositionConstants.kPositionHighY)
      // };
    
    //Clear pre-sets? - AJ

    m_extendPrimaryMotor.burnFlash();
    m_extendSecondaryMotor.burnFlash();
    m_winchMotor.burnFlash();
    
    
  }



  public double[] thingsFromCartesianPoint(double horizontalPosition, double verticalPosition)  {
    //Finding the required extension for a point
    double desiredExtension = (
        (Math.sqrt(
            Math.pow(horizontalPosition, 2) + 
            Math.pow(verticalPosition, 2) -
            ElevatorConstants.EquationConstants.kIntakeOffsetFromArm)) -
        ElevatorConstants.EquationConstants.kArmLength
    );

    //Finding the requried rotation for a point.
    //atan returns radians from -pi/2 to pi/2
    double angleTheta = Math.atan(verticalPosition / horizontalPosition);
    double anglePhi = Math.atan(
                            ElevatorConstants.EquationConstants.kIntakeOffsetFromArm / 
                            (desiredExtension) + ElevatorConstants.EquationConstants.kArmLength
                        );

    double desiredAngle = (angleTheta + anglePhi);

    //Finding the requried length of rope to give the correct angle
    double valueB = Math.sqrt(
                        Math.pow(ElevatorConstants.EquationConstants.kPulleySidewaysOffset, 2) + 
                        Math.pow(ElevatorConstants.EquationConstants.kPulleyHeight, 2)
                    );
    double valueC = Math.sqrt(
                        Math.pow(ElevatorConstants.EquationConstants.kArmOffsetFromRope, 2) + 
                        Math.pow(ElevatorConstants.EquationConstants.kArmLength, 2)
                    );

    double angleAlpha = Math.atan(
                            ElevatorConstants.EquationConstants.kPulleyHeight / 
                            ElevatorConstants.EquationConstants.kPulleySidewaysOffset
                        );
    double angleBeta = Math.atan(
                            ElevatorConstants.EquationConstants.kArmOffsetFromRope / 
                            ElevatorConstants.EquationConstants.kArmLength
                        );
    double angleGamma = (Math.PI - angleAlpha - angleBeta - desiredAngle);

    double desiredRopeLength = Math.sqrt(
                                Math.pow(valueC, 2) + Math.pow(valueB, 2) - 
                                (2 * valueB * valueC * Math.cos(angleGamma))
                            );

    return new double[] {desiredExtension, desiredRopeLength, desiredAngle};
}


public double winchEncoderTicks(double desiredRopeLength, double currentRopeLength) {
    //TODO: any way to get current rope length, besides storing as a class variable?

    double reqChange = desiredRopeLength - currentRopeLength;
    double reqWinchRevs = (reqChange / (Math.PI * ElevatorConstants.EquationConstants.kWinchDiameter));
    double reqMotorRevsRotation = (reqWinchRevs * ElevatorConstants.EquationConstants.kGearRatioRotation);
    double reqEncoderTicksRotation = (reqMotorRevsRotation * ElevatorConstants.EquationConstants.kEncoderTicksPerRevRotation);
    return reqEncoderTicksRotation;
}

public void moveElevatorUnSafe(double extendSpeed, double winchSpeed) {
  m_extendPrimaryMotor.set(extendSpeed);
  m_winchMotor.set(winchSpeed);
}
  

public void MoveElevator(double extendSpeed, double winchSpeed){
  double winchSet = 0;
  double extendSet = 0;
  double winchPos = m_winchMotorBuiltInEncoder.getPosition();
  double extendPos = m_extendLeftMotorBuiltInEncoder.getPosition();

  if (winchPos < WinchConstants.k45DegreesRots) {
    // Dowwn
    if (extendPos > ExtenderConstants.kMaxExtentionFlat) {
      extendSet = extendSpeed;
      winchSet = winchSpeed;
      // Below Max Extend
    } else {
      // Above Max Extend
      if (winchSpeed <= 0){
        // Retracting winch
        winchSet = winchSpeed;
      } else {
        // Extending winch
        winchSet = .25;
      }
      if (extendSpeed <= 0) {
      // Extending
      // Don't move
      extendSet = 0;
    } else {
      // Retracting
      extendSet = extendSpeed;
    }
  }
 } else {
    // Above reaching angle, so okay to extend as much as we want
    winchSet = winchSpeed;
    extendSet = extendSpeed;
  }

  if (Math.abs(winchSpeed) < .1) {
    if (winchZeroSpeedBool) {
      if (winchPos < winchZeroSpeedDouble) {
        winchSet = .1* ((-72-winchPos)/(-72)); // Modulate force w/ angle
      } else {
        winchSet = 0;
      }
    
  } else {
    winchZeroSpeedBool = true;
    winchZeroSpeedDouble = winchPos;
  }
} else {
  winchZeroSpeedBool = false;
}

  if (Math.abs(extendSpeed) < .1) {
    if (extendZeroSpeedBool) {
      if (extendPos > extendZeroSpeedDouble) {
        extendSet = -.05 * ((-72-winchPos)/(-72)); // modulate force w/ angle
      } else {
        extendSet = 0;
      }
    
  } else {
    extendZeroSpeedBool = true;
    extendZeroSpeedDouble = extendPos;
  }
}  else {
  extendZeroSpeedBool = false;
}

  m_extendPrimaryMotor.set(extendSet);
  m_winchMotor.set(winchSet);
}

  


// public void moveToPoint(int desiredPos) {
//   double desExt = positionSetpoints[desiredPos][0];
//   double desRope = positionSetpoints[desiredPos][1];
//   double desAngle = positionSetpoints[desiredPos][2];

//   m_extendPIDController.setReference(desExt, ControlType.kPosition);
//   m_winchPIDController.setReference(desAngle, ControlType.kPosition);
// }

  @Override
  public void periodic() {
    
    table.getEntry("Extend Encoder Rotations").setDouble(m_extendLeftMotorBuiltInEncoder.getPosition());
    table.getEntry("Winch Encoder Rotations").setDouble(m_winchMotorBuiltInEncoder.getPosition());
    table.getEntry("Extend Setpoint").setDouble(extendDesiredSetpoint);
    table.getEntry("Winch Setpoint").setDouble(winchDesiredSetpoint);

    // Don't put moving things in periodic, use it for updating inputs. Make a seperate command for moving things.
  //   m_extendPIDController.setReference(positionSetpoints[][], ControlType.kPosition);
    //still need to get correct values and get things from which button.
    //the other PID's will work the same way
  }

  public double getAbsPos() {
     return m_ThruBoreEncoder.getAbsolutePosition() - ElevatorConstants.kThruBoreOffset;
  }

  public void zeroEncoder() {
    m_winchMotorBuiltInEncoder.setPosition(0);
  }

  public void moveElevatorAuto(double desWinch, double desExt) {
    extendDesiredSetpoint = desExt;
    winchDesiredSetpoint = desWinch;

    m_extendPIDController.setReference(desExt, ControlType.kPosition);
    m_winchPIDController.setReference(desWinch, ControlType.kPosition);
  }

  public void stayElevatorAuto() {
    m_extendPIDController.setReference(extendDesiredSetpoint, ControlType.kPosition);
    m_winchPIDController.setReference(winchDesiredSetpoint, ControlType.kPosition);
  }

  public void moveElevatorAutoProfile(double desWinch, double desExt) {
    double extendSet = m_extendProfiledPIDController.calculate(m_extendLeftMotorBuiltInEncoder.getPosition(), desExt);
    double winchSet = m_winchProfiledPIDController.calculate(m_winchMotorBuiltInEncoder.getPosition(), desWinch);

    m_extendPrimaryMotor.set(extendSet);
    m_winchMotor.set(winchSet);
  }


  public void logInit() {


  }
}