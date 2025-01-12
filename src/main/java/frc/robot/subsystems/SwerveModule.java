// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.CANCoderUtil;
import frc.lib.OnboardModuleState;
import frc.lib.CTREConfigs;
import frc.lib.SwerveModuleConstants;
import frc.lib.CANCoderUtil.CCUsage;
import frc.robot.Constants;
import frc.robot.Robot;

/** Add your docs here. */
public class SwerveModule {
    public int moduleNumber;
    public double m_angleKP;
    public double m_angleKI;
    public double m_angleKD;
    public double m_angleKFF;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private SparkMax angleMotor;
    private SparkMax driveMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
  
    private CANcoder angleEncoder;

    private SparkClosedLoopController driveController;
    private SparkClosedLoopController angleController;

   private final SimpleMotorFeedforward feedforward =
   new SimpleMotorFeedforward(
       Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);
    //creates a feedforward for the swerve drive. feedforward does 90% of the work, estimating stuff
    //PID fixes the error

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.m_angleKP = moduleConstants.angleKP;
        this.m_angleKI = moduleConstants.angleKI;
        this.m_angleKD = moduleConstants.angleKD;
        this.m_angleKFF = moduleConstants.angleKFF;
        angleOffset = moduleConstants.angleOffset;
        //this.?
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getClosedLoopController();
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getClosedLoopController();
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(driveEncoder.getVelocity(),  getAngle()); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(driveEncoder.getPosition(),  getAngle()); 
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Custom optimize command, since default WPILib optimize assumes continuous controller which
        // REV supports this now so dont have to worry with rev, but need some funky configs i dont want to do
        //have to be sad with falcons but thats what you get for giving money to Tony
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
        
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            //when not taking feedback
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            driveMotor.set(percentOutput);
        }
        else {
            driveController.setReference(
                desiredState.speedMetersPerSecond,
                com.revrobotics.spark.SparkBase.ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                feedforward.calculate(desiredState.speedMetersPerSecond)
            );
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        //the ? and : are a shorthand for an if-else loop
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; 
        angleController.setReference(
            angle.getDegrees(),
            SparkBase.ControlType.kPosition
        );
        lastAngle = angle;
    }

    private void resetToAbsolute() {
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        integratedAngleEncoder.setPosition(absolutePosition); //may need to change 
    }
    
    public Rotation2d getCanCoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
        //return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }
    
    private void configAngleEncoder(){   

        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());     

        CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);

        //angleEncoder.optimizeBusUtilization();
    }

    private void configAngleMotor(){
        SparkMaxConfig angConfig = new SparkMaxConfig();

        angConfig
            .inverted(Constants.SwerveConstants.angleInvert)
            .smartCurrentLimit(Constants.SwerveConstants.angleContinuousCurrentLimit)
            .voltageCompensation(Constants.SwerveConstants.voltageComp)
            .idleMode(Constants.SwerveConstants.angleNeutralMode);
        angConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(m_angleKP, m_angleKI, m_angleKD, m_angleKFF);
        angConfig.encoder
            .positionConversionFactor(Constants.SwerveConstants.angleConversionFactor);
        angConfig.signals
            .busVoltagePeriodMs(100);
        //limits can bus usage

        angleMotor.configure(angConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        Timer.delay(1.0);
        //resets to the cancoder
        resetToAbsolute();
    }

    private void configDriveMotor(){    
        SparkMaxConfig drvConfig = new SparkMaxConfig();

        drvConfig
            .inverted(Constants.SwerveConstants.driveInvert)
            .idleMode(Constants.SwerveConstants.driveNeutralMode)
            .voltageCompensation(Constants.SwerveConstants.voltageComp)
            .smartCurrentLimit(Constants.SwerveConstants.driveContinuousCurrentLimit);
        drvConfig.encoder
            .positionConversionFactor(Constants.SwerveConstants.driveConversionPositionFactor)
            .velocityConversionFactor(Constants.SwerveConstants.driveConversionVelocityFactor);
        drvConfig.closedLoop
            .pidf(Constants.SwerveConstants.driveKP, Constants.SwerveConstants.driveKI, Constants.SwerveConstants.driveKD, Constants.SwerveConstants.driveKFF)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        drvConfig.signals
            .busVoltagePeriodMs(100);

        //burns to spark max
        driveMotor.configure(drvConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //resets encoder position to 0
        driveEncoder.setPosition(0.0);
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }
    









  

}
