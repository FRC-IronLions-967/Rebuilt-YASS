// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface Module_IO {
    @AutoLog
    public static class Module_IO_Inputs{

    }

    public default void updateInputs(Module_IO_Inputs inputs) {}

} 
