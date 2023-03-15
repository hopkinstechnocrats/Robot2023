// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Auto.AutoRoutines;

import java.util.List;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  //private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ManipulatorSubsystem m_manipulator = new ManipulatorSubsystem();
  private final AutoRoutines m_autoRoutines = new AutoRoutines(m_robotDrive, m_elevator, m_manipulator);
  public Pose2d zeroPose = new Pose2d();
  // private final SingleModuleTestFixture singleModuleTestFixture = new SingleModuleTestFixture();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("Robot Container");
  private NetworkTable autoTable = inst.getTable("Auto");
  private SendableChooser<Command> m_autoChooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    CameraServer.startAutomaticCapture();
    
    m_autoChooser.setDefaultOption("Balance Auto", m_autoRoutines.autoBalance());
    m_autoChooser.addOption("Drive Forward", m_autoRoutines.driveStraightAuto(3.0, 0.0));
    m_autoChooser.addOption("Place Auto", m_autoRoutines.placeAuto());
    m_autoChooser.addOption("Two Place Auto", m_autoRoutines.twoPlaceAuto());
    m_autoChooser.addOption("Solely Place", m_autoRoutines.solelyPlace());
    m_autoChooser.addOption("Place And Balance", m_autoRoutines.placeAndBalance());
    m_autoChooser.addOption("Leave And Balance", m_autoRoutines.outAndBalance());
    m_autoChooser.addOption("null", null);
    SmartDashboard.putData(m_autoChooser);

    table.getEntry("winch Desired Position").setDouble(0);
    table.getEntry("extend Desired Position").setDouble(0);
      //Start logging
      DataLogManager.start();
      DataLog log = DataLogManager.getLog();
      //Log Driverstation Inputs
      DriverStation.startDataLog(log);
    // Configure the button bindings
    configureButtonBindings();
    // loggables.add(driv);

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -3*m_driverController.getLeftY(),
                    -3*m_driverController.getLeftX(),
                    3*m_driverController.getRightX(),
               m_driverController.getRightTriggerAxis()), m_robotDrive));

    m_elevator.setDefaultCommand(
        new RunCommand(
            () ->
            m_elevator.stayElevatorAuto()
                       , m_elevator)
    );
    // singleModuleTestFixture.setDefaultCommand(
    //         new RunCommand(
    //             () -> 
    //                 singleModuleTestFixture.setState(
    //                     m_driverController.getLeftY(), m_driverController.getRightY()),
    //             singleModuleTestFixture)
    // );

    m_manipulator.setDefaultCommand(new RunCommand(() -> m_manipulator.NoSpin(), m_manipulator));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
      JoystickButton AButton = new JoystickButton(m_driverController, 1);
      JoystickButton BButton = new JoystickButton(m_driverController, 2);
      JoystickButton XButton = new JoystickButton(m_driverController, 3);
      JoystickButton YButton = new JoystickButton(m_driverController, 4);
      
      // 
      POVButton DPadTop = new POVButton(m_driverController, 90);
      POVButton DPadRight = new POVButton(m_driverController, 180);
      POVButton DPadBottom = new POVButton(m_driverController, 270);
      POVButton DPadLeft = new POVButton(m_driverController, 0);

      AButton.onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
      BButton.onTrue(new InstantCommand(() -> m_robotDrive.resetOdometry(zeroPose)));

      //Rotates all wheels to 45 degrees relative to the frame
      XButton.whileTrue(new RunCommand(() -> m_robotDrive.defence(), m_robotDrive));
      // DButton.whenPressed(new InstantCommand(() -> singleModuleTestFixture.setAngle(new Rotation2d(0, -1))));
      
      //AutoRotate to desired heading
      
      DPadTop.whileTrue(new RunCommand(() -> m_robotDrive.autoRotate(-1*m_driverController.getLeftY(),
      -1*m_driverController.getLeftX(),
      0,
      m_driverController.getRightTriggerAxis()), m_robotDrive));

      DPadRight.whileTrue(new RunCommand(() -> m_robotDrive.autoRotate(-1*m_driverController.getLeftY(),
      -1*m_driverController.getLeftX(),
      Math.PI/2,
      m_driverController.getRightTriggerAxis()), m_robotDrive));

      DPadBottom.whileTrue(new RunCommand(() -> m_robotDrive.autoRotate(-1*m_driverController.getLeftY(),
      -1*m_driverController.getLeftX(),
      Math.PI,
      m_driverController.getRightTriggerAxis()), m_robotDrive));


      DPadLeft.whileTrue(new RunCommand(() -> m_robotDrive.autoRotate(-1*m_driverController.getLeftY(),
      -1*m_driverController.getLeftX(),
      -Math.PI/2,
      m_driverController.getRightTriggerAxis()), m_robotDrive));

      
      JoystickButton OLBButton = new JoystickButton(m_operatorController, 5);
      JoystickButton OAButton = new JoystickButton(m_operatorController, 1);
      JoystickButton ORBButton = new JoystickButton(m_operatorController, 6);
      JoystickButton OXButton = new JoystickButton(m_operatorController, 3);
      JoystickButton OYButton = new JoystickButton(m_operatorController, 4);
      JoystickButton OBButton = new JoystickButton(m_operatorController, 2);

      //Spin Cone out
      ORBButton.whileTrue(new RunCommand(() -> m_manipulator.SpinCone(false), m_manipulator)); 
      //Spin Cone in
      OLBButton.whileTrue(new RunCommand(() -> m_manipulator.SpinCone(true), m_manipulator)); 
      //Spin Cube out
      OXButton.whileTrue(new RunCommand(() -> m_manipulator.SpinCube(false), m_manipulator)); 
      //Spin Cube in
      OYButton.whileTrue(new RunCommand(() -> m_manipulator.SpinCube(true), m_manipulator)); 

      OAButton.whileTrue(new RunCommand(() -> m_elevator.moveElevatorAuto(AutoConstants.kConeSSPounceWinch, AutoConstants.kConeSSPounceExt), m_elevator));

      OBButton.whileTrue(new RunCommand(() -> m_elevator.moveElevatorAuto(0, 0), m_elevator));


      POVButton ODPadTop = new POVButton(m_operatorController, 90);
      ODPadTop.whileTrue(new RunCommand(() -> m_elevator.moveElevatorAuto(AutoConstants.kHighScoreWinch, AutoConstants.kHighScoreExt), m_elevator));
      
      POVButton ODPadRight = new POVButton(m_operatorController, 180);
      ODPadRight.whileTrue(new RunCommand(() -> m_elevator.moveElevatorAuto(AutoConstants.kMidScoreWinch, AutoConstants.kMidScoreExt), m_elevator));
      
      POVButton ODPadBottom = new POVButton(m_operatorController, 270);
      ODPadBottom.whileTrue(new RunCommand(() -> m_elevator.moveElevatorAuto(AutoConstants.kGroundPickWinch, AutoConstants.kGroundPickExt), m_elevator));
      
      POVButton ODPadLeft = new POVButton(m_operatorController, 0);
      ODPadLeft.whileTrue(new RunCommand(() -> m_elevator.moveElevatorAuto(AutoConstants.kConeSubStationWinch, AutoConstants.kConeSubStationExt), m_elevator));
      
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  public Command getAutonomousCommand() {

    // return m_autoChooser.getSelected();
    return new RunCommand(() -> m_elevator.moveElevatorAuto(table.getEntry("winch Desired Position").getDouble(0), 
    table.getEntry("extend Desired Position").getDouble(0)), m_elevator);
   /* EXAMPLE AUTO CODE
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxSpeedMetersPerSecond)
                //,
                //                AutoConstants.kMaxAccelerationMetersPerSecondSquared

            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(.4, 1), new Translation2d(.8, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(1.2, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
 
    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, 0));
    */ 
  }
}
