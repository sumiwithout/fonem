// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralSubsystemConstants.ElevatorSetpoints;
import frc.robot.subsystems.ElevatorSubsytem;
import frc.robot.subsystems.ElevatorSubsytem.hightes;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class level4commnd extends Command {
  private static final ElevatorSubsytem elvator = ElevatorSubsytem.getInstance();

  /** Creates a new level4commnd. */
  public level4commnd() {

    addRequirements(elvator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elvator.setSetpointCommand(hightes.levcel2);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("blueberries", "eleavator  finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elvator.getencoder() >= ElevatorSetpoints.kLevel2;

  }
}
