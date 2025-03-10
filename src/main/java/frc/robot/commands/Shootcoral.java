// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shootcoral extends Command {
  private static final Coral coralshooting = Coral.getinstance();
  private Timer m_timer;
  /** Creates a new ShootforAuto. */
  public Shootcoral() {
    addRequirements(coralshooting);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
                m_timer = new Timer();
                m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  coralshooting.shooting();
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralshooting.stopshooting();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > .3;
  }
}
