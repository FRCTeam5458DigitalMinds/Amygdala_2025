package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;

public class Climber extends SubsystemBase {
    private double stage1 = (7.51419507 + 6.488);
    private double stage2 = (39.1089286);
    private double[] m_setPoints = {0, stage1, stage2};

    private int climb_ID1 = ClimbConstants.climb_ID1;
    private int climb_ID2 = ClimbConstants.climb_ID2;

    private final SparkClosedLoopController climbController;
    private RelativeEncoder climbEncoder;

    private SparkMax climbMotor_1;
    private SparkMax climbMotor_2;
    private int current_stage = 0;

  public Climber() {
    climbMotor_1 = new SparkMax(climb_ID1, MotorType.kBrushless);
    climbMotor_2 = new SparkMax(climb_ID2, MotorType.kBrushless);

    climbEncoder = climbMotor_1.getEncoder();
    climbController = climbMotor_1.getClosedLoopController();


    //not sure if this is right syntax w/ funcs
    //need to do more testing; look here for errors. 11/12/24
    //push code then open rev client to double check values (make them weird to triple check)

    SparkMaxConfig climbConfig = new SparkMaxConfig();

    //period
    climbConfig
        .smartCurrentLimit(80)
        .idleMode(IdleMode.kBrake)
        .follow(climb_ID1);
    climbConfig.closedLoop  
        .pidf(Constants.ClimbConstants.kP, Constants.ClimbConstants.kI, Constants.ClimbConstants.kD, Constants.ClimbConstants.FF);
 
    MAXMotionConfig climbControllerConfig = new MAXMotionConfig();
    climbControllerConfig
        .maxAcceleration(Constants.ClimbConstants.max_accel)
        .maxVelocity(Constants.ClimbConstants.min_vel)
        .allowedClosedLoopError(Constants.ClimbConstants.allowed_error);

    climbMotor_1.configure(climbConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    climbMotor_2.configure(climbConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void toSetPoint(int setPoint) 
  {
    /*
    if (setPoint == 1)
    {
      climbMotor_1.set(0.05); 
    }
    else
    {
      climbMotor_1.set(0);
    } */

    //HERE
    climbController.setReference(m_setPoints[setPoint], SparkMax.ControlType.kMAXMotionPositionControl);
    SmartDashboard.putString("DB/String 4", Double.toString(m_setPoints[setPoint]));
    SmartDashboard.putString("DB/String 6", Double.toString(climbMotor_1.get()));
    current_stage = setPoint;
  }

  public int getStage()
  {
    return current_stage;
  }

  public double getInches()
  {
    //CHANGE TO CONVERSION 
    double current_inches = climbEncoder.getPosition() * (1/16.3) * (1.5992 * 3.14159);
    return current_inches;
  }

  public double getEncoder()
  {
    return climbEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
 
  /* protected void interrupted()
  {
   
  } */

}