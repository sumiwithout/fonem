// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class HangSubsystem extends SubsystemBase {
// find number

private static final HangSubsystem hang = new HangSubsystem();
  public static final HangSubsystem getinstance(){
    return hang;
  }
  private final SparkMax hangmotor = new SparkMax(18, MotorType.kBrushless);
  private SparkClosedLoopController hanvontorller = hangmotor.getClosedLoopController();
  private RelativeEncoder hangEncoder = hangmotor.getEncoder();

  public HangSubsystem() {
hangmotor.configure(Configs.Hangsubsystem.armhang,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
hangEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
  
    // This method will be called once per scheduler run
  }


    private void sethang(double position) {
    hanvontorller.setReference(position, ControlType.kPosition);
  }

   public Command backhome() {
    return this.run(
        () -> {
          sethang(0);
        });

  }

  
  public Command hang() {
    return this.run(
        () -> {
          sethang(20);
        });
  }
}
