// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private static final double kAngleSetpoint = 0.0;
  private static final double kP = 0.005; // propotional turning constant
  AHRS ahrs;
  private final WPI_TalonSRX m_leftLeader = new WPI_TalonSRX(11);
  private final WPI_VictorSPX m_leftFollower = new WPI_VictorSPX(12);

  private final WPI_TalonSRX m_rightLeader = new WPI_TalonSRX(13);
  private final WPI_VictorSPX m_rightFollower = new WPI_VictorSPX(14);

  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftLeader, m_leftFollower);
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightLeader, m_rightFollower);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors,m_rightMotors);

    // The left-side drive encoder
  private final Encoder m_leftEncoder =
  new Encoder(
      DriveConstants.kLeftEncoderPorts[0],
      DriveConstants.kLeftEncoderPorts[1],
      DriveConstants.kLeftEncoderReversed);

// The right-side drive encoder
private final Encoder m_rightEncoder =
  new Encoder(
      DriveConstants.kRightEncoderPorts[0],
      DriveConstants.kRightEncoderPorts[1],
      DriveConstants.kRightEncoderReversed);


  public Drivetrain() {
    m_rightMotors.setInverted(true);

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(DriveConstants.kleftEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.krightEncoderDistancePerPulse);

    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      ahrs = new AHRS(SPI.Port.kMXP); 
    }   catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }
    if(ahrs.isConnected()){
      ahrs.calibrate();
      while(ahrs.isCalibrating())
        ;
      ahrs.reset();
      ahrs.zeroYaw();
    }
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("left encoder", m_leftEncoder.getDistance());
    SmartDashboard.putNumber("right encoder", m_rightEncoder.getDistance());
    SmartDashboard.putNumber("yaw", ahrs.getYaw());
  }
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }
  public void driveForward(double fwd){
    double turningValue = -1 * (kAngleSetpoint - ahrs.getYaw()) * kP;
    // Invert the direction of the turn if we are going backwards
    turningValue = Math.copySign(turningValue, fwd);
    arcadeDrive(fwd, 0);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Gets the average distance of the TWO encoders.
   *
   * @return the average of the TWO encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }
}

