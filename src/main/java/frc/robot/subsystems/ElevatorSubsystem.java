// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.TestFixtureConstants;
import lib.Loggable;
import badlog.lib.BadLog;


public class ElevatorSubsystem extends SubsystemBase implements Loggable {
  /** Creates a new SingleModuleTestFixture. */
  public static final CANSparkMaxLowLevel.MotorType kBrushless;

  private final CANSparkMax extendLeftMotor = new CANSparkMax(1, kBrushless);
  private final CANSparkMax extendRightMotor = new CANSparkMax(2, kBrushless);
  private final CANSparkMax winchMotor = new CANSparkMax(3, kBrushless);
  private final SparkMaxAbsoluteEncoder encoder = new SparkMaxAbsoluteEncoder(winchMotor, );
  
      

  
  @Override
  public void periodic() {
    
  }

  public void logInit() {
    

  }
}