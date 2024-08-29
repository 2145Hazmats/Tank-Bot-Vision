// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PhotonVisionConstants;

public class Drivetrain extends SubsystemBase {

  private DifferentialDrive m_differentialDrive;
  private DifferentialDriveKinematics m_driveKinematics;
  private DifferentialDrivePoseEstimator m_driveOdometry;

  private final CANSparkMax leftMotor = new CANSparkMax(DrivetrainConstants.MOTOR_LEFT_ID, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(DrivetrainConstants.MOTOR_RIGHT_ID, MotorType.kBrushless);
  private RelativeEncoder leftMotorEncoder = leftMotor.getEncoder();
  private RelativeEncoder rightMotorEncoder = rightMotor.getEncoder();

  private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  private PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
  private PhotonPipelineResult result = null;
  private PhotonTrackedTarget target = null;
  private PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
      AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
      //Calculates a new robot position estimate by combining all visible tag corners.
      //If using MULTI_TAG_PNP_ON_COPROCESSOR, must configure the AprilTagFieldLayout properly in the UI.
      //https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/multitag.html#multitag-localization
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      camera,
      PhotonVisionConstants.ROBOT_TO_CAMERA);
  private EstimatedRobotPose latestRobotPose = null;

  private PIDController driveController = new PIDController(DrivetrainConstants.DRIVE_P, 0, 0);
  private PIDController rotController = new PIDController(DrivetrainConstants.ANGULAR_P, 0, 0);

  private final Field2d m_field = new Field2d();

  public Drivetrain() {
    rightMotorEncoder.setPositionConversionFactor(DrivetrainConstants.ENCODER_CONVERSION_FACTOR);
    leftMotorEncoder.setPositionConversionFactor(DrivetrainConstants.ENCODER_CONVERSION_FACTOR);
    rightMotorEncoder.setPosition(0);
    leftMotorEncoder.setPosition(0);

    leftMotor.setInverted(true);
    rightMotor.setInverted(false);

    gyro.calibrate();
    gyro.reset();

    m_differentialDrive = new DifferentialDrive(leftMotor, rightMotor);
    m_driveKinematics = new DifferentialDriveKinematics(DrivetrainConstants.TRACK_WIDTH);
    m_driveOdometry = new DifferentialDrivePoseEstimator(
        m_driveKinematics,
        new Rotation2d(gyro.getAngle()),
        0,
        0,
        new Pose2d()
    );
    // We can implement the standard deviation of vision to make pose estimation more accurate using a Matrix<N3, N1>, but it is complicated.

    SmartDashboard.putData("Field", m_field);
  }

  public double getGyroAngle() {
    return gyro.getAngle();
  }

  public Rotation2d getGyroRotation2d() {
    return new Rotation2d(Units.degreesToRadians(gyro.getAngle()));
  }

  public Pose2d getPose2d() {
    return m_driveOdometry.getEstimatedPosition();
  }
  
  public Command drive(DoubleSupplier leftY, DoubleSupplier rightX, DoubleSupplier rightY, Supplier<String> driveType) {
    return Commands.run(() -> {
      switch (driveType.get()) {
        case "arcadeDrive":
          m_differentialDrive.arcadeDrive(leftY.getAsDouble(), rightX.getAsDouble());
          break;
        case "tankDrive":
          m_differentialDrive.tankDrive(leftY.getAsDouble(), rightY.getAsDouble());
          break;
        case "curvatureDrive1":
          m_differentialDrive.curvatureDrive(leftY.getAsDouble(), rightX.getAsDouble(), false);
          break;
        case "curvatureDrive2":
          m_differentialDrive.curvatureDrive(leftY.getAsDouble(), rightX.getAsDouble(), true);
          break;
      }
      SmartDashboard.putNumber("Controller Left Y", leftY.getAsDouble());
      SmartDashboard.putNumber("Controller Right X", rightX.getAsDouble());
      SmartDashboard.putNumber("Controller Right Y", rightY.getAsDouble());
      SmartDashboard.putString("Selected Drive Type", driveType.get());
    }, this);
  }

  // Rotates towards an april tag (uses degrees NOT radians)
  public Command rotateToTarget(DoubleSupplier leftY, DoubleSupplier rightX) {
    return Commands.run(
        () -> {
          // Update Vision results
          result = camera.getLatestResult();
          // Get the current best target
          target = result.getBestTarget();

          // If there is a valid target, rotate towards it using PID
          if (result.hasTargets()) { // or if (target == null)
            m_differentialDrive.arcadeDrive(
                leftY.getAsDouble(),
                -rotController.calculate(target.getYaw(), 0) //calcuate a PID output based on the target's yaw (in degrees)
            );
            SmartDashboard.putNumber("Rot Offset", target.getYaw());
            // For "Vision Rot Speed", positive = clockwise rotation. arcadeDrivealso uses positive = clockwise rotation
            SmartDashboard.putNumber("Vision Rot Speed", -rotController.calculate(target.getYaw(), 0));
          }
          else {
            m_differentialDrive.arcadeDrive(
                leftY.getAsDouble(),
                rightX.getAsDouble()
            );
          }
        },
        this
    );
  }

  //Drive towards an april tag, and stop a set meters away from the april tag
  public Command driveToTarget(DoubleSupplier leftY, DoubleSupplier rightX, double desiredRange) {
    return Commands.run(
        () -> {
          // Update Vision results
          result = camera.getLatestResult();
          // Get the current best target
          target = result.getBestTarget();

          // if there is a target, drive towards it using PID
          if (result.hasTargets()) {
            // calculate the distance in meters of the april tag (range == distance)
            double range = PhotonUtils.calculateDistanceToTargetMeters(
              PhotonVisionConstants.CAMERA_HEIGHT,
              PhotonVisionConstants.TARGET_HEIGHT,
              PhotonVisionConstants.CAMERA_PITCH_RADIANS,
              Units.degreesToRadians(target.getPitch())
            );
            m_differentialDrive.arcadeDrive(
                driveController.calculate(range, desiredRange), //calculate a PID output using the april tag's distance and the desired distance (meters)
                rightX.getAsDouble()
            );
            SmartDashboard.putNumber("Distance To Target", range);
            SmartDashboard.putNumber("Desired Distance", desiredRange);
            SmartDashboard.putNumber("Vision Drive Speed", driveController.calculate(range, desiredRange));
          }
          else {
            m_differentialDrive.arcadeDrive(
                leftY.getAsDouble(),
                rightX.getAsDouble()
            );
          }
        },
        this
    );
  }

  // Drives and rotates towards and april tag. Stops a set distance away. For more documentation, see rotateToTarget and driveToTarget
  public Command driveRotateToTarget(DoubleSupplier leftY, DoubleSupplier rightX, double desiredRange) {
    return Commands.run(
        () -> {
          result = camera.getLatestResult();
          target = result.getBestTarget();

          if (result.hasTargets()) {
            double range = PhotonUtils.calculateDistanceToTargetMeters(
              PhotonVisionConstants.CAMERA_HEIGHT,
              PhotonVisionConstants.TARGET_HEIGHT,
              PhotonVisionConstants.CAMERA_PITCH_RADIANS,
              Units.degreesToRadians(target.getPitch())
            );
            m_differentialDrive.arcadeDrive(
                driveController.calculate(range, desiredRange),
                -rotController.calculate(target.getYaw(), 0)
            );
            SmartDashboard.putNumber("Distance To Target", range);
            SmartDashboard.putNumber("Desired Distance", desiredRange);
            SmartDashboard.putNumber("Vision Drive Speed", driveController.calculate(range, desiredRange));
            SmartDashboard.putNumber("Rot Offset", target.getYaw());
            SmartDashboard.putNumber("Vision Rot Speed", -rotController.calculate(target.getYaw(), 0));
          }
          else {
            m_differentialDrive.arcadeDrive(
                leftY.getAsDouble(),
                rightX.getAsDouble()
            );
          }
        },
        this
    );
  }

  // Add a vision measurement to the robot's odometry
  public void addVisionPose2d(Pose2d visionPose2d, double timestampSeconds) {
    m_driveOdometry.addVisionMeasurement(visionPose2d, timestampSeconds); //Timer.getFPGATimestamp());
  }

  @Override
  public void periodic() {
    // Update Vision results
    result = camera.getLatestResult();

    // Try to update latestRobotPose with a new EstimatedRobotPose and add the vision Pose2d to drivetrains odometry
    try {
      latestRobotPose = poseEstimator.update(result).get();
      addVisionPose2d(latestRobotPose.estimatedPose.toPose2d(), latestRobotPose.timestampSeconds);
    } catch (Exception e) { // catch = catching an exception, java.util.Optional.get() throws NoSuchElementException if no value is present
      latestRobotPose = null;
    }

    // Update drive odometry without using vision
    m_driveOdometry.updateWithTime(
        Timer.getFPGATimestamp(),
        new Rotation2d(Units.degreesToRadians(gyro.getAngle())),
        -leftMotorEncoder.getPosition(),
        -rightMotorEncoder.getPosition()
    );
    
    SmartDashboard.putNumber("Rot", gyro.getAngle());
    SmartDashboard.putNumber("Bot X", getPose2d().getX());
    SmartDashboard.putNumber("Bot Y", getPose2d().getY());
    if (latestRobotPose != null) { SmartDashboard.putNumber("Vision Rot", Units.radiansToDegrees(latestRobotPose.estimatedPose.getRotation().getAngle())); }
    if (latestRobotPose != null) { SmartDashboard.putNumber("Vision X", latestRobotPose.estimatedPose.getX()); }
    if (latestRobotPose != null) { SmartDashboard.putNumber("Vision Y", latestRobotPose.estimatedPose.getY()); }
    if (latestRobotPose != null) { SmartDashboard.putNumber("Vision Timestamp", latestRobotPose.timestampSeconds); }
    m_field.setRobotPose(m_driveOdometry.getEstimatedPosition());
  }
}
