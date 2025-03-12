// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandStadiaController;
import frc.robot.commands.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private CommandSwerveDrivetrain drivebase;
  private SwerveRequest.FieldCentricFacingAngle drive;
  private double tagID = -1;

  public AlignToReefTagRelative(boolean isRightScore, CommandSwerveDrivetrain drivebase) {
    xController = new PIDController(Constants.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(Constants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(Constants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(Constants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(Constants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(Constants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? Constants.Y_SETPOINT_REEF_ALIGNMENT : -Constants.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(Constants.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("");
      SmartDashboard.putNumber("x", postions[2]);

      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("xspee", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);

    drivebase.setControl(drive
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withVelocityX(xSpeed) // Drive forward with negative Y(forward)
          .withVelocityY(ySpeed) // Drive left with negative X (left)
          .withTargetDirection(Rotation2d.fromDegrees(rotValue)));

    if (!rotController.atSetpoint() ||
        !yController.atSetpoint() ||
        !xController.atSetpoint()) {
      stopTimer.reset();
    }
     
     else {
      drivebase.setControl(drive
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withVelocityX(0) // Drive forward with negative Y(forward)
          .withVelocityY(0) // Drive left with negative X (left)
          .withTargetDirection(Rotation2d.fromDegrees(0)));
    }

      SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.setControl(drive
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withVelocityX(0) // Drive forward with negative Y(forward)
          .withVelocityY(0) // Drive left with negative X (left)
          .withTargetDirection(Rotation2d.fromDegrees(0)));
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
  }
}