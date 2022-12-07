// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTime extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_seconds;
  private final double m_speed;
  private final Timer m_timer;
  /** Creates a new DriveDistance. */
  public DriveTime(double seconds, double speed, Drivetrain drive) {
    m_seconds = seconds;
    m_speed = speed;
    m_drive = drive;
    m_timer = new Timer();
    addRequirements(m_drive);
  }
  /** Creates a new DriveTime. */
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Timer", m_timer.get());
    m_drive.driveForward(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > m_seconds;
  }
}
