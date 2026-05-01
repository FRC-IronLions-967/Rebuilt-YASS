// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
// Import relevant classes.
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



// Example SwerveDrive class
public class Swerve extends SubsystemBase {

    // Attributes
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    AHRS gyro; // Gyroscope.
    Module_IO[] swerveModules; // Swerve modules.
    Module_IO_InputsAutoLogged inputs[];
    // Constructor
    public Swerve(Module_IO modulefl, Module_IO modulefr, Module_IO modulebl, Module_IO modulebr) {
    
        inputs[0] = new Module_IO_InputsAutoLogged();
        inputs[1] = new Module_IO_InputsAutoLogged();
        inputs[2] = new Module_IO_InputsAutoLogged();
        inputs[3] = new Module_IO_InputsAutoLogged();
        swerveModules[0] = modulefl;
        swerveModules[1] = modulefr;
        swerveModules[2] = modulebl;
        swerveModules[3] = modulebr;
    
        // Create SwerveDriveKinematics object
        // 10in from center of robot to center of wheel.
        // 10in is converted to meters to work with object.
        // Translation2d(x,y) == Translation2d(front, left)
        kinematics = new SwerveDriveKinematics(
            new Translation2d(Units.inchesToMeters(10), Units.inchesToMeters(10)), // Front Left
            new Translation2d(Units.inchesToMeters(10), Units.inchesToMeters(-10)), // Front Right
            new Translation2d(Units.inchesToMeters(-10), Units.inchesToMeters(10)), // Back Left
            new Translation2d(Units.inchesToMeters(-10), Units.inchesToMeters(-10))  // Back Right
        );
        
        gyro = new AHRS(NavXComType.kMXP_SPI, (byte) 100);

        // Create the SwerveDriveOdometry given the current angle, the robot is at x=0, r=0, and heading=0
        odometry = new SwerveDriveOdometry(
            kinematics,
            Rotation2d.fromDegrees(gyro.getAngle()), // returns current gyro reading as a Rotation2d
            new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()}
            // Front-Left, Front-Right, Back-Left, Back-Right
        );
            
    }

    @Override
    public void periodic() {
        for (int i = 0; i <= 3; i++) {
            swerveModules[i].updateInputs(inputs[i]);
        }
    }
    
    // Simple drive function
    public void drive(DoubleSupplier x_Speed, DoubleSupplier y_Speed, DoubleSupplier rotation_Speed) {
        // Create test ChassisSpeeds going X = 14in, Y=4in, and spins at 30deg per second.
        Translation2d speed_Vector = new Translation2d(x_Speed.getAsDouble(), y_Speed.getAsDouble());
        ChassisSpeeds speeds = new ChassisSpeeds(
            speed_Vector.getX() *Drive_Constants.maxSpeedMetersPerSec, 
            speed_Vector.getY() *Drive_Constants.maxSpeedMetersPerSec, 
            rotation_Speed.getAsDouble() * 3.84644
            );
        
        // Get the SwerveModuleStates for each module given the desired speeds.
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(speeds);
        // Output order is Front-Left, Front-Right, Back-Left, Back-Right
        
        swerveModules[0].setState(swerveModuleStates[0]);
        swerveModules[1].setState(swerveModuleStates[1]);
        swerveModules[2].setState(swerveModuleStates[2]);
        swerveModules[3].setState(swerveModuleStates[3]);
    }
    
    // Fetch the current swerve module positions.
    public SwerveModulePosition[] getCurrentSwerveModulePositions() {
        return new SwerveModulePosition[]{
            new SwerveModulePosition(inputs[0].drive_Position, inputs[0].turn_Angle), // Front-Left
            new SwerveModulePosition(inputs[1].drive_Position, inputs[1].turn_Angle), // Front-Right
            new SwerveModulePosition(inputs[2].drive_Position, inputs[2].turn_Angle), // Back-Left
            new SwerveModulePosition(inputs[3].drive_Position, inputs[3].turn_Angle)  // Back-Right
        };
    }
                               
    public void updateOdometry() {
        // Update the odometry every run.
        odometry.update(Rotation2d.fromDegrees(gyro.getAngle()), getCurrentSwerveModulePositions());
    }
    
}
