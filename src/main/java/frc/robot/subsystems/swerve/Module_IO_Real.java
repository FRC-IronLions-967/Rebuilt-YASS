// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;


public class Module_IO_Real implements Module_IO {

    private SparkMax driveMotor;
    private SparkMax turnMotor;
    private AbsoluteEncoder absoluteEncoder;
    private SparkClosedLoopController drivingController;
    private SparkClosedLoopController turningController;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder turnEncoder;
    protected SparkMaxConfig driveConfig;
    
    public Module_IO_Real(int driveMotorCANID, int turnMotorCANID)
    {
        driveMotor = new SparkMax(driveMotorCANID, MotorType.kBrushless);
        turnMotor = new SparkMax(turnMotorCANID, MotorType.kBrushless);
        absoluteEncoder = turnMotor.getAbsoluteEncoder();
        
        // Get the PID Controllers
        drivingController = driveMotor.getClosedLoopController();
        turningController = turnMotor.getClosedLoopController();
        
        // Get the encoders
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();
        
        // Continue configuration here..
        
        // turning Motor Configuration
        driveConfig = new SparkMaxConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Drive_Constants.driveMotorCurrentLimit)
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(Drive_Constants.driveEncoderPositionFactor)
        .velocityConversionFactor(Drive_Constants.driveEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig
        .closedLoop
        .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
        .pid(
            Drive_Constants.driveKp.get(), 0.0,
            Drive_Constants.driveKd.get());
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / Drive_Constants.odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
        driveMotor.configure(
                driveConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);


    // Configure turn motor
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(Drive_Constants.turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Drive_Constants.turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .absoluteEncoder
        .inverted(Drive_Constants.turnEncoderInverted)
        .positionConversionFactor(Drive_Constants.turnEncoderPositionFactor)
        .velocityConversionFactor(Drive_Constants.turnEncoderVelocityFactor)
        .averageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(Drive_Constants.turnPIDMinInput, Drive_Constants.turnPIDMaxInput)
        .pid(Drive_Constants.turnKp.get(), 0.0, Drive_Constants.turnKd.get());
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / Drive_Constants.odometryFrequency))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
        turnMotor.configure(
                turnConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        }


    @Override
    public void updateInputs(Module_IO_Inputs inputs) {
        inputs.drive_Position = driveEncoder.getPosition();
        inputs.turn_Angle = Rotation2d.fromDegrees(turnEncoder.getPosition());
    }
       
    /**
    Set the swerve module state.
    @param state The swerve module state to set.
    */
    @Override
    public void setState(SwerveModuleState state)
    {
          turningController.setSetpoint(state.angle.getDegrees(), ControlType.kPosition);
          drivingController.setSetpoint(state.speedMetersPerSecond, ControlType.kVelocity);
    }

}
