package frc.robot;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import frc.robot.subsystems.*;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Autos extends SequentialCommandGroup{
  /** Example static factory for an autonomous command. */
  private Autos(int cases, DriveSubsystem d_Drive) {
    System.out.printf("autos selection: %d/n", cases);
    switch (cases) {
        case 0:
        movementAuto(d_Drive);
        break;
    default:
        doNothingAuto(d_Drive);
        break;
    }
  }

  public void doNothingAuto(DriveSubsystem d_Drive) {}

  public void movementAuto(DriveSubsystem d_Drive) {
    System.out.println("move auto");  

    String trajectoryJSON = "PathWeaver/output/Test.wpilib.json";
    Trajectory trajectory = new Trajectory(); 
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        System.out.println("Path " + trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        };
    // An example trajectory to follow.  All units in meters.
    Trajectory Trajectory = trajectory;
    
    var thetaController =
    new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    RamseteCommand ramseteCommand =
    new RamseteCommand(
        Trajectory,
        d_Drive::getPose,
        new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
                Constants.DriveConstants.ksVolts,
                Constants.DriveConstants.kvVoltSecondsPerMeter,
                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics,
            d_Drive::getWheelSpeeds,
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            d_Drive::tankDriveVolts,
            d_Drive
    );
        
    addCommands(
        new InstantCommand(() -> d_Drive.resetOdometry(Trajectory.getInitialPose())),
        ramseteCommand
    );
  }
}
