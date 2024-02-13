package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.SwerveSubsystem;
public class zeroheading extends SequentialCommandGroup{
    public zeroheading(
        SwerveSubsystem m_swerve

    ){
    addCommands(
        new InstantCommand(() -> m_swerve.zeroHeading()));
    }
}