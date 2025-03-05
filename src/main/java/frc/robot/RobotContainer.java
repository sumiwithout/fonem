// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.ElevatorSubsytem;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.ElevatorSubsytem.hightes;

public class RobotContainer {
      private final ElevatorSubsytem m_ElevatorSubsytem = ElevatorSubsytem.getInstance();
      private final AlgaeSubsystem m_algaeSubsystem = AlgaeSubsystem.getinstance();
      private final HangSubsystem m_hangSubsystem = HangSubsystem.getinstance();
        private final Coral shooter = Coral.getinstance();
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer(){
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
                SmartDashboard.putData("Auto Mode", autoChooser);
                registerNamedCommands();
        configureBindings();
    }
// test test on fonem grave 
//alright it works
    private void configureBindings() {
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );




            shooter.setDefaultCommand(new RunCommand(()-> shooter.stopshooting(), shooter));
            




        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick2.b().onTrue(m_ElevatorSubsytem.setSetpointCommand(hightes.stattion));

        // A Button -> Elevator/Arm to level 2 position
        joystick2.a().onTrue(m_ElevatorSubsytem.setSetpointCommand(hightes.levcel2));
    
        // X Button -> Elevator/Arm to level 3 position
        joystick2.x().onTrue(m_ElevatorSubsytem.setSetpointCommand(hightes.level3));
    
        // // Y Button -> Elevator/Arm to level 4 position
        joystick2.y().onTrue(m_ElevatorSubsytem.setSetpointCommand(hightes.level4));
        
        joystick2.rightTrigger().whileTrue(new RunCommand(()-> shooter.shooting(), shooter));
        joystick2.leftTrigger().whileTrue(new RunCommand(()-> shooter.intake(), shooter));
        joystick2.leftBumper().onTrue(m_algaeSubsystem.kickalgeCommand());
        joystick2.rightBumper().onTrue(m_algaeSubsystem.backhome());
        // joystick2.start().onTrue(m_hangSubsystem.hang());

    }
    public double getSimulationTotalCurrentDraw() {
        // for each subsystem with simulation
        return m_ElevatorSubsytem.getSimulationCurrentDraw();
      }
      public void registerNamedCommands(){
        NamedCommands.registerCommand("L4", new InstantCommand(() ->m_ElevatorSubsytem.setSetpointCommand(hightes.levcel2)));
      }
    

    public Command getAutonomousCommand() {

        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
