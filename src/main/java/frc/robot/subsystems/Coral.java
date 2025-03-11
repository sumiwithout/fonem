// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.Elevatorsubsystem;
import frc.robot.Configs.shoot;
import frc.robot.Constants.CoralSubsystemConstants.ElevatorSetpoints;
import frc.robot.subsystems.ElevatorSubsytem.hightes;

public class Coral extends SubsystemBase {
  private final ElevatorSubsytem m_ElevatorSubsytem = ElevatorSubsytem.getInstance();
  public enum state{
    shoot, 
    stop, 
    intake, 
    level1, 
    IDLE
} 
public static final Coral shootting = new Coral();
public static Coral getinstance(){
  return shootting;
}
state current = state.IDLE;

  private SparkMax coralshoot = new SparkMax(12, MotorType.kBrushless);
  private SparkMax followshoot = new SparkMax(9, MotorType.kBrushless);
  private ColorSensorV3 yes;
  // private Rev2mDistanceSensor distance = new Rev2mDistanceSensor(Port.kOnboard);
  /** Creates a new shooter. */
  public Coral() {
    yes = new ColorSensorV3(Port.kOnboard);
 coralshoot.configure(shoot.coralshoot, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followshoot.configure(shoot.followshoot, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void shooting(){
    current = state.shoot;
  
  }
  public void stopshooting(){
    current = state.stop;
  }
  public void level1shoot(){
    current = state.level1;
  }
  public void intake(){
    current = state.intake;
  }
  /**
   * @param powerL left coral shooter power
   * @param powerR right coral shooter power
   */
  public void setpower(double coral,double follow){
    coralshoot.set(coral);
    followshoot.set(follow);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (current) {
      case shoot:
      coralshoot.set(-.3);
      followshoot.set(-.3);
        break;
    
      case stop:
      coralshoot.set(0);
      
      followshoot.set(0);
      break;
      case level1:
      coralshoot.set(-.2);
      
      followshoot.set(-.1);
      break;
      case intake:
      coralshoot.set(-.1);
      followshoot.set(-.1);
      break;
     case IDLE:


     break;
    }
  }

}
