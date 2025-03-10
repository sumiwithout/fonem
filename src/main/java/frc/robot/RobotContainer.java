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
import frc.robot.commands.ShootforAuto;
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

      
        private final Coral shooter = Coral.getinstance();
       private ShootforAuto level1shoot = new ShootforAuto();
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
    private double multipler = .85;
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer(){
        NamedCommands.registerCommand("shoot", new InstantCommand(()-> shooter.setpower(-.2,-.1)));
        NamedCommands.registerCommand("timmer",  level1shoot.withTimeout(5));

        NamedCommands.registerCommand("stop", new InstantCommand(()-> shooter.stopshooting()));
        NamedCommands.registerCommand("L2", new InstantCommand(() ->m_ElevatorSubsytem.setSetpointCommand(hightes.levcel2)));

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
                SmartDashboard.putData("Auto Mode", autoChooser);

                configureBindings();
    }

    private void configureBindings() {
        



        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed*multipler) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed*multipler) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate*multipler) // Drive counterclockwise with negative X (left)
            )
        );
        
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        joystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(0.5))
        );
        joystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(-.5))
        );
        shooter.setDefaultCommand(new RunCommand(()-> shooter.stopshooting(), shooter));
          





        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));




        // // reset the field-centric heading on left bumper press



        // player one type shit
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        joystick.rightBumper().onTrue(m_algaeSubsystem.removefromreef());
        joystick.leftBumper().whileTrue(new RunCommand(()-> shooter.level1shoot(), shooter));
        joystick.rightTrigger().whileTrue(new RunCommand(()-> shooter.shooting(), shooter));
        joystick.leftTrigger().whileTrue(new RunCommand(()-> shooter.intake(), shooter));
        drivetrain.registerTelemetry(logger::telemeterize);
        joystick.b().onTrue(m_algaeSubsystem.backhome());




        // player 2
        joystick2.b().onTrue(m_ElevatorSubsytem.setSetpointCommand(hightes.stattion));
        joystick2.a().onTrue(m_ElevatorSubsytem.setSetpointCommand(hightes.levcel2));
        joystick2.x().onTrue(m_ElevatorSubsytem.setSetpointCommand(hightes.level3));
        joystick2.y().onTrue(m_ElevatorSubsytem.setSetpointCommand(hightes.level4));
        joystick2.rightBumper().onTrue(m_algaeSubsystem.backhome());
        joystick2.leftBumper().onTrue(m_algaeSubsystem.removefromreef());
        joystick2.rightTrigger().onTrue(m_algaeSubsystem.inprocessor());
        joystick2.leftTrigger().onTrue(m_algaeSubsystem.shootprocesor());

    }

    public double getSimulationTotalCurrentDraw() {
        // for each subsystem with simulation
        return m_ElevatorSubsytem.getSimulationCurrentDraw();
      }
      public void registerNamedCommands(){
      

      }
    

    public Command getAutonomousCommand() {

        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
