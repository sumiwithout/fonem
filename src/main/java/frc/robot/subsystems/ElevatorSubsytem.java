package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Configs.Elevatorsubsystem;
import frc.robot.Constants.CoralSubsystemConstants.ElevatorSetpoints;
import frc.robot.Constants.SimulationRobotConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorSubsytem extends SubsystemBase{
    public enum hightes{
            level, 
            stattion, 
            levcel2, 
            level3, 
            level4
    }

    
    
    private SparkMax elevatormotor = new SparkMax(14, MotorType.kBrushless);
    private SparkMax followmotor = new SparkMax(11, MotorType.kBrushless);

    // private SparkMax followmotor = new SparkMax(100, MotorType.kBrushless);
    private SparkClosedLoopController elactorcontorller = elevatormotor.getClosedLoopController();
    private RelativeEncoder elevatEncoder = elevatormotor.getEncoder();

   private DCMotor elevatorMotorModel = DCMotor.getNeoVortex(1);
  private SparkMaxSim elevatorMotorSim;
  private SparkLimitSwitchSim elevatorLimitSwitchSim;
 private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          elevatorMotorModel,
          SimulationRobotConstants.kElevatorGearing,
          SimulationRobotConstants.kCarriageMass,
          SimulationRobotConstants.kElevatorDrumRadius,
          SimulationRobotConstants.kMinElevatorHeightMeters,
          SimulationRobotConstants.kMaxElevatorHeightMeters,
          true,
          SimulationRobotConstants.kMinElevatorHeightMeters,
          0.0,
          0.0);
     
          private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 25, 0);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d(
              "Elevator",
              SimulationRobotConstants.kMinElevatorHeightMeters,
              90));

       private boolean wasResetByButton = false;
  private boolean wasResetByLimit = false;
  private double elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
  private static final ElevatorSubsytem m_elevator = new ElevatorSubsytem();
  public static ElevatorSubsytem getInstance(){
    return m_elevator;
  }
public ElevatorSubsytem(){
    elevatormotor.configure(Elevatorsubsystem.elevatorconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followmotor.configure(Elevatorsubsystem.followmotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatEncoder.setPosition(0);


    elevatorMotorSim = new SparkMaxSim (elevatormotor, elevatorMotorModel);
    elevatorLimitSwitchSim = new SparkLimitSwitchSim(elevatormotor, false);
SmartDashboard.putData("Elavator",m_mech2d);

}
private void moveToSetpoint(){
//  will work on it more later
elactorcontorller.setReference(elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
}
public void zeroTheElevatorOnLimitSwitch(){
    if(!wasResetByButton && elevatormotor.getReverseLimitSwitch().isPressed()){
        elevatEncoder.setPosition(0);
        wasResetByLimit=true;
    }
    else if(!elevatormotor.getReverseLimitSwitch().isPressed()){
        wasResetByLimit=false;

    }
}
 private void zeroOnUserButton() {
    if (!wasResetByButton && RobotController.getUserButton()) {
      // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
      // constant zeroing while pressed
      wasResetByButton = true;
      elevatEncoder.setPosition(0);
    } else if (!RobotController.getUserButton()) {
      wasResetByButton = false;
    }
  }
   public Command setSetpointCommand(hightes setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case stattion:
              elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
              break;
            case level:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
              break;
            case levcel2:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
              break;
            case level3:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
              break;
            case level4:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
              break;
          }
        });
        
  }

@Override
public void periodic(){

  moveToSetpoint();
    zeroTheElevatorOnLimitSwitch();
    zeroOnUserButton();
     SmartDashboard.putNumber("levator/Target Position", elevatorCurrentTarget);
    SmartDashboard.putNumber("Elevator/Actual Position", elevatEncoder.getPosition());

    m_elevatorMech2d.setLength(
    SimulationRobotConstants.kMinElevatorHeightMeters
          + SimulationRobotConstants.kMaxElevatorHeightMeters
              * (elevatEncoder.getPosition() / SimulationRobotConstants.kElevatorGearing)
              * (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI));
}
public void setCurrenttarget(int num){
    elevatorCurrentTarget = num;
}
public double getSimulationCurrentDraw() {
  return m_elevatorSim.getCurrentDrawAmps();
}
public double getencoder(){
    return elevatEncoder.getPosition();
}
@Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(elevatormotor.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Update sim limit switch
    elevatorLimitSwitchSim.setPressed(m_elevatorSim.getPositionMeters() == 0);

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // Iterate the elevator and arm SPARK simulations
    elevatorMotorSim.iterate(
        ((m_elevatorSim.getVelocityMetersPerSecond()
                    / (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI))
                * SimulationRobotConstants.kElevatorGearing)
            * 60.0,
        RobotController.getBatteryVoltage(),
        0.02);
  

    // SimBattery is updated in Robot.java
  }

}
