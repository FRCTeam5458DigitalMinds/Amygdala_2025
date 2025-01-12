package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class GroundIntake extends SubsystemBase {
  //PID setpoints of ideal encoder values
  private double deployPosition = -36;
  private double climbingPosition = -13.104;
  private double origin = -1.; //-1.5
  private double ejectPosition = -27;

  private double[] m_setPoints = {origin, deployPosition, climbingPosition, ejectPosition};
  
  //declaring intake motors/encoders
  private final SparkClosedLoopController intakeController;
  private RelativeEncoder intakeEncoder;
  
  private SparkMax intakeMotor;
  private SparkMax rollerMotor;
  private SparkMaxConfig intakeConfig;
  private SparkMaxConfig rollerConfig;

  public GroundIntake() {

    //motor definitions
    intakeMotor = new SparkMax(Constants.IntakeConstants.intake_ID, MotorType.kBrushless);
    rollerMotor = new SparkMax(Constants.IntakeConstants.roller_ID, MotorType.kBrushless);

    //delcaring & resetting intake slap down encoder
    intakeEncoder = intakeMotor.getEncoder();

    //configuring the motors
    intakeConfig = new SparkMaxConfig();
    intakeConfig
        .smartCurrentLimit(30)
        .idleMode(IdleMode.kBrake);
    intakeConfig.closedLoop
        .pidf(Constants.IntakeConstants.kP,Constants.IntakeConstants.kI,Constants.IntakeConstants.kD,Constants.IntakeConstants.FF);
    
    rollerConfig = new SparkMaxConfig();
    rollerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(35);

    //intake slap down PID declaration and initial config of values
    intakeController = intakeMotor.getClosedLoopController();

    MAXMotionConfig intakeControllerConfig = new MAXMotionConfig();
    intakeControllerConfig
        //.setfeed(intakeEncoder)
        .maxAcceleration(Constants.IntakeConstants.max_accel)
        .maxVelocity(Constants.IntakeConstants.max_vel)
        .allowedClosedLoopError(Constants.IntakeConstants.allowed_error);

  }

  //moves intake slapdown to a certain encoder value from preset array
  //setPoint passed in as index
  public void toSetPoint(int setPoint) 
  {
    SmartDashboard.putNumber("Intake setpoint", setPoint);
    //FIX SET REFERENCE
    intakeController.setReference(
      m_setPoints[setPoint], 
      SparkMax.ControlType.kMAXMotionPositionControl);
  }

  //sets the speed of the intake rollers
  public void setRollers(double OutputPercent)
  {
      OutputPercent /= 100.;
      rollerMotor.set(-OutputPercent);
  }

  //returns the output current of the intake roller motor
  public double voltageOutput()
  {
    SmartDashboard.putNumber("roller output current", rollerMotor.getOutputCurrent());
    
    return rollerMotor.getOutputCurrent();
  }

  //returns the intake slap down motor encoder position
  public double getPos()
  {
    return intakeEncoder.getPosition();
  }
  public void setZero()
  {
    intakeEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}