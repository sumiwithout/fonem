package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;


public final class Configs {
  
        public static double pvalue =.08;
        
        public static double dvale = 0.05;
   
  public static final class Elevatorsubsystem{
        public static final SparkMaxConfig elevatorconfig = new SparkMaxConfig();
        public static final SparkMaxConfig followmotor = new SparkMaxConfig();
    static {
        elevatorconfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
        elevatorconfig.limitSwitch.reverseLimitSwitchEnabled(true).reverseLimitSwitchType(Type.kNormallyOpen);
        
        followmotor.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
        followmotor.limitSwitch.reverseLimitSwitchEnabled(true).reverseLimitSwitchType(Type.kNormallyOpen);
        followmotor.follow(14,true);
        elevatorconfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(pvalue, 0, dvale,0.001).outputRange(-1, 1).maxMotion.maxVelocity(4200).maxAcceleration(6000).allowedClosedLoopError(.5);
        // elevatorconfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // .p(pvalue)
        // .d(dvale)
        // .outputRange(-1, 1)
        // .maxMotion
        // .maxVelocity(4200)
        // .maxAcceleration(6000)
        // .allowedClosedLoopError(.5);
        

  }
}

public static final class shoot{
  public static final SparkMaxConfig coralshoot = new SparkMaxConfig();
  public static final SparkMaxConfig followshoot = new SparkMaxConfig();
  static {
    coralshoot.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    coralshoot.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);

  }
  
  }
  public static final class AlgaeSubsystem {
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();
    public static final SparkMaxConfig armConfig = new SparkMaxConfig();

    static {
      // Configure basic setting of the arm motor
      armConfig.smartCurrentLimit(40);
      armConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control. We don't need to pass a closed
          // loop slot, as it will default to slot 0.
          .p(0.1)
          .outputRange(-0.5, 0.5);

      // Configure basic settings of the intake motor
      intakeConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    }
  }
 public static final class Hangsubsystem{
  public static final SparkMaxConfig armhang = new SparkMaxConfig();
  static{
    armhang.smartCurrentLimit(40);
    armhang
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(0.1)
        .outputRange(-0.5, 0.5);
  }

 }
}
