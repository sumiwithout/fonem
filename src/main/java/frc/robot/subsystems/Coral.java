// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.Elevatorsubsystem;
import frc.robot.Configs.shoot;

public class Coral extends SubsystemBase {
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
  
  /** Creates a new shooter. */
  public Coral() {
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (current) {
      case shoot:
        coralshoot.set(-1);
        followshoot.set(-1);
        break;
    
      case stop:
      coralshoot.set(0);
      
      followshoot.set(0);
      break;
      case level1:
      coralshoot.set(-.3);
      
      followshoot.set(-.2);
      break;
      case intake:
      coralshoot.set(.1);
      followshoot.set(.1);
      break;
     case IDLE:


     break;
    }
  }
}
