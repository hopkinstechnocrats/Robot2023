// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.TestFixtureConstants;
import lib.Loggable;
import badlog.lib.BadLog;


public class ElevatorSubsystem extends SubsystemBase implements Loggable {
  /** Creates a new SingleModuleTestFixture. */

  private final WPI_TalonFX driveMotor = new WPI_TalonFX(TestFixtureConstants.kDriveMotorPort);
  private final WPI_TalonFX turningMotor = new WPI_TalonFX(TestFixtureConstants.kTurningMotorPort);
  private final CANCoder encoder = new CANCoder(TestFixtureConstants.kCANCoderID);
  
      

  
  @Override
  public void periodic() {
    
  }

  public void logInit() {
    

  }
}