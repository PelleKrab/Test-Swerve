// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.stream.Collectors;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.PivotConfig.PivotId;
import frc.robot.subsystems.drive.commands.Drive;
import frc.robot.utilities.FileWriteUtil;
import frc.robot.utilities.Logger;

/** Represents a swerve drive style drivetrain. */
public class DriveSubsystem extends SubsystemBase {
  public static final double kMaxSpeed = 4.2672; // 3 meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // 1/2 rotation per second

  public static final double x = 0.257175; // 10.125"
  public static final double y = 0.32146875; // 12.65625"

  // OK
  private final Translation2d frontLeftLocation = new Translation2d(x, y);
  private final Translation2d frontRightLocation = new Translation2d(x, -y);
  private final Translation2d backLeftLocation = new Translation2d(-x, y);
  private final Translation2d backRightLocation = new Translation2d(-x, -y);

  // OK
  private final SwerveModule frontLeft = new SwerveModule(PivotConfig.getConfig(PivotId.FL));
  private final SwerveModule frontRight = new SwerveModule(PivotConfig.getConfig(PivotId.FR));
  private final SwerveModule backLeft = new SwerveModule(PivotConfig.getConfig(PivotId.BL));
  private final SwerveModule backRight = new SwerveModule(PivotConfig.getConfig(PivotId.BR));

  // OK
  private static final ADIS16448_IMU gyro = new ADIS16448_IMU();
  private final Rotation2d angle = new Rotation2d(Math.toRadians(gyro.getAngle()));

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, angle);

  public Rotation2d rotate2D(){
    Rotation2d angle2D = new Rotation2d(Math.toRadians(gyro.getAngle()));
    return angle2D;
  }

  public DriveSubsystem() {
    gyro.reset();
  }

  public void initDefaultCommand() {
    setDefaultCommand(new Drive(this, true));
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, angle);
  }

  public void resetGyro() {
    gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, 
                rotate2D())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);

    // Logger.log(frontLeft.getPositionY() + "");

    // FileWriteUtil.log(
    // Stream.of(frontLeft, frontRight, backLeft, backRight)
    // .map(sw -> sw.turningEncoder)
    // .flatMap(res -> Stream.of(
    // res.getMinV(),
    // res.getMaxV(),
    // res.getResolver().getVoltage(),
    // res.getD()
    // )
    // )
    // Stream.of(
    // TimeUtil.getTime(),
    // swerveModuleStates[0].angle.getDegrees(),
    // swerveModuleStates[1].angle.getDegrees(),
    // swerveModuleStates[2].angle.getDegrees(),
    // swerveModuleStates[3].angle.getDegrees(),
    // // frontLeft.getState().angle.getDegrees(),
    // // frontRight.getState().angle.getDegrees(),
    // // backLeft.getState().angle.getDegrees(),
    // // backRight.getState().angle.getDegrees()
    // frontLeft.getPosition(),
    // frontRight.getPosition(),
    // backLeft.getPosition(),
    // backRight.getPosition()

    // )
    // .map(s -> String.format("%f", s))
    // .collect(Collectors.joining(","))
    // );
    // Logger.log(frontLeft.getPosition() + "")
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxSpeed);
    frontLeft.setDesiredStateAuto(desiredStates[0]);
    frontRight.setDesiredStateAuto(desiredStates[1]);
    backLeft.setDesiredStateAuto(desiredStates[2]);
    backRight.setDesiredStateAuto(desiredStates[3]);

    FileWriteUtil.log(
        Stream.of(frontLeft, frontRight, backLeft, backRight)
            .flatMap(sw -> Stream.of(
                // sw.turningEncoder.getMinV(),
                // sw.turningEncoder.getMaxV(),
                // sw.turningEncoder.getResolver().getVoltage(),
                sw.turningEncoder.getD(),
                sw.getVelocity(),
                sw.getPositionX(),
                sw.getPositionY(),
                sw.echo(odometry.getPoseMeters().getX()),
                sw.echo(odometry.getPoseMeters().getY()),
                sw.desiredState.speedMetersPerSecond,
                sw.desiredState.angle.getDegrees()))
            .map(s -> String.format("%f", s))
            .collect(Collectors.joining(",")));
    // Logger.log(frontLeft.getPosition() + "")
  }

  public void pointWheels(double angle) {
    frontLeft.setAngleD(angle);
    frontRight.setAngleD(angle);
    backLeft.setAngleD(angle);
    backRight.setAngleD(angle);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    odometry.update(
        rotate2D(),
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
  }

  public Pose2d getPose() {
    updateOdometry();
    Logger.log("x position: " + odometry.getPoseMeters().getX() + " y position: " + odometry.getPoseMeters().getY());
    return odometry.getPoseMeters();
  }

  public void resetEncoders() {
    Stream.of(frontLeft, frontRight, backLeft, backRight).forEach(SwerveModule::resetEncoder);
  }
}
