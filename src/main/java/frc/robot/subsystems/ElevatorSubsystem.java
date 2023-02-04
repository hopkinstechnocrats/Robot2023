// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.TestFixtureConstants;
import lib.Loggable;
import badlog.lib.BadLog;


public class ElevatorSubsystem extends SubsystemBase implements Loggable {
  /** Creates a new SingleModuleTestFixture. */

  private final CANSparkMax m_extendPrimaryMotor = new CANSparkMax(Constants.ElevatorConstants.ExtenderConstatants.kPrimaryMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_extendSecondaryMotor = new CANSparkMax(Constants.ElevatorConstants.ExtenderConstatants.kSecondaryMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_winchMotor = new CANSparkMax(Constants.ElevatorConstants.WinchConstants.kMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
  //Asumes Hall Effect Encoder
  private final RelativeEncoder m_extendLeftMotorBuiltInEncoder = m_extendPrimaryMotor.getEncoder();
  //private final RelativeEncoder m_extendRightMotorBuiltInEncoder = m_extendRightMotor.getEncoder();
  private final RelativeEncoder m_winchMotorBuiltInEncoder = m_winchMotor.getEncoder();
      
  //Using Left Motor As Primary Motor
  private final SparkMaxPIDController m_extendPIDController = m_extendPrimaryMotor.getPIDController();
  private final SparkMaxPIDController m_winchPIDController = m_winchMotor.getPIDController();

  public ElevatorSubsystem(){
    //Clear pre-sets? - AJ
    m_extendPrimaryMotor.restoreFactoryDefaults();
    m_extendSecondaryMotor.restoreFactoryDefaults();
    m_winchMotor.restoreFactoryDefaults();

    m_extendPIDController.setFeedbackDevice(m_extendLeftMotorBuiltInEncoder);
    m_winchPIDController.setFeedbackDevice(m_winchMotorBuiltInEncoder);

    m_extendPIDController.setP(Constants.ElevatorConstants.ExtenderConstatants.kP);
    m_extendPIDController.setI(Constants.ElevatorConstants.ExtenderConstatants.kI);
    m_extendPIDController.setD(Constants.ElevatorConstants.ExtenderConstatants.kD);
    m_extendPIDController.setFF(Constants.ElevatorConstants.ExtenderConstatants.kFF);
    m_extendPIDController.setIZone(Constants.ElevatorConstants.ExtenderConstatants.kIz);
    m_extendPIDController.setOutputRange(Constants.ElevatorConstants.ExtenderConstatants.kMinOutput, Constants.ElevatorConstants.ExtenderConstatants.kMaxOutput);

    m_winchPIDController.setI(Constants.ElevatorConstants.WinchConstants.kI);
    m_winchPIDController.setP(Constants.ElevatorConstants.WinchConstants.kP);
    m_winchPIDController.setD(Constants.ElevatorConstants.WinchConstants.kD);
    m_winchPIDController.setFF(Constants.ElevatorConstants.WinchConstants.kFF);
    m_winchPIDController.setIZone(Constants.ElevatorConstants.WinchConstants.kIz);
    m_winchPIDController.setOutputRange(Constants.ElevatorConstants.WinchConstants.kMinOutput, Constants.ElevatorConstants.WinchConstants.kMaxOutput);

  }
  
  @Override
  public void periodic() {
    
  }

  public void logInit() {
    

  }
}