package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;


public final class Configs {
        
   
  public static final class Elevatorsubsystem{
        public static final SparkMaxConfig elevatorconfig = new SparkMaxConfig();
        // public static final SparkMaxConfig followmotor = new SparkMaxConfig();
    static {
        elevatorconfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
        elevatorconfig.limitSwitch.reverseLimitSwitchEnabled(true).reverseLimitSwitchType(Type.kNormallyOpen);
        
        // followmotor.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
        // followmotor.limitSwitch.reverseLimitSwitchEnabled(true).reverseLimitSwitchType(Type.kNormallyOpen);
        // followmotor.inverted(true).follow(20);


        elevatorconfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(.1)
        .outputRange(-1, 1)
        .maxMotion
        .maxVelocity(4200)
        .maxAcceleration(6000)
        .allowedClosedLoopError(.5);
        

  }
}
}
