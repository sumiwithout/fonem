package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;


public class Autos extends Command{
  private final SendableChooser<Command> autonChooser;
      public Autos(){
        autonChooser = new SendableChooser<Command>();
      
         autonChooser.setDefaultOption("nope", new InstantCommand());
        
       SmartDashboard.putData("Auton Chooser", autonChooser);
     
       
      }
      public Command getSelected() {
        return autonChooser.getSelected();
      }
     
       
    
        private void buildAuto(String autoName) {
            Command autoCommand = AutoBuilder.buildAuto(autoName);
            autonChooser.addOption(autoName, autoCommand);
          }
          //comment
}
