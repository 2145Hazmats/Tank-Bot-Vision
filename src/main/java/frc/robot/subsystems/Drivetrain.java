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
  // Declaring DifferentialDrive, kinematics, and odometry for a basic non-vision tank robot
  private DifferentialDrive m_differentialDrive;
  private DifferentialDriveKinematics m_driveKinematics;
  private DifferentialDrivePoseEstimator m_driveOdometry;
  // Motors, encoders, and the gyro
  private final CANSparkMax leftMotor = new CANSparkMax(DrivetrainConstants.MOTOR_LEFT_ID, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(DrivetrainConstants.MOTOR_RIGHT_ID, MotorType.kBrushless);
  private RelativeEncoder leftMotorEncoder = leftMotor.getEncoder();
  private RelativeEncoder rightMotorEncoder = rightMotor.getEncoder();

  private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  // PhotonVision objects
  private PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
  private PhotonPipelineResult result = null;
  private PhotonTrackedTarget target = null;
  // PhotonVision objects used in vision localization
  private PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
      AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
      //Calculates a new robot position estimate by combining all visible tag corners.
      //If using MULTI_TAG_PNP_ON_COPROCESSOR, must configure the AprilTagFieldLayout properly in the UI.
      //https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/multitag.html#multitag-localization
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      camera,
      PhotonVisionConstants.ROBOT_TO_CAMERA);
  private EstimatedRobotPose latestRobotPose = null;
  // PIDControllers used in vision
  private PIDController driveController = new PIDController(DrivetrainConstants.DRIVE_P, 0, 0);
  private PIDController rotController = new PIDController(DrivetrainConstants.ANGULAR_P, 0, 0);
  // Field to display on SmartDashboard
  private final Field2d m_field = new Field2d();

  public Drivetrain() {
    // Encoder conversion and setup
    rightMotorEncoder.setPositionConversionFactor(DrivetrainConstants.ENCODER_CONVERSION_FACTOR);
    leftMotorEncoder.setPositionConversionFactor(DrivetrainConstants.ENCODER_CONVERSION_FACTOR);
    rightMotorEncoder.setPosition(0);
    leftMotorEncoder.setPosition(0);
    // Motor setup
    leftMotor.setInverted(true);
    rightMotor.setInverted(false);
    // Gyro setup
    gyro.calibrate();
    gyro.reset();
    // Initializing DifferentialDrive, kinematics, and odometry for a basic non-vision tank robot
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

  // Return the gyro's angle
  public double getGyroAngle() {
    return gyro.getAngle();
  }

  // Returns the gyro's Rotation2d
  public Rotation2d getGyroRotation2d() {
    return new Rotation2d(Units.degreesToRadians(gyro.getAngle()));
  }

  // Returns the robot's Pose2D
  public Pose2d getPose2d() {
    return m_driveOdometry.getEstimatedPosition();
  }
  
  // Master drive command for Drivetrain. Uses controller input and a supplied "driveType" string as parameters
  public Command drive(DoubleSupplier leftY, DoubleSupplier rightX, DoubleSupplier rightY, Supplier<String> driveType) {
    return Commands.run(() -> {
      // switch case statement to call arcadeDrive() with different arguments based on "driveType"
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
        default:
          System.out.println("Invalid driveType selected!");
          break;
      }
      // Log information to SmartDashboard
      SmartDashboard.putNumber("Controller Left Y", leftY.getAsDouble());
      SmartDashboard.putNumber("Controller Right X", rightX.getAsDouble());
      SmartDashboard.putNumber("Controller Right Y", rightY.getAsDouble());
      SmartDashboard.putString("Selected Drive Type", driveType.get());
    }, this);
  }

  // Rotates towards an april tag using 2D processing
  public Command rotateToTarget(DoubleSupplier leftY, DoubleSupplier rightX) {
    return Commands.run(() -> {
      // Get the latest camera results and target
      result = camera.getLatestResult();
      target = result.getBestTarget();

      // Only use vision information if there is a valid target
      if (result.hasTargets()) { // or "if (target != null)"
        // Calls arcadeDrive using the output of the rotation PID controller and joystick input
        m_differentialDrive.arcadeDrive(
            leftY.getAsDouble(),
            -rotController.calculate(target.getYaw(), 0) //calcuate a PID output based on the target's yaw (in degrees)
        );
        // Log information to SmartDashboard 
        SmartDashboard.putNumber("Rot Offset", target.getYaw());
        // For target.getYaw(), positive = clockwise rotation. This is because in arcadeDrive, positive = clockwise rotation
        SmartDashboard.putNumber("Vision Rot Speed", -rotController.calculate(target.getYaw(), 0));
      }
      // If there are no valid targets, call arcadeDrive with joystick input
      else {
        m_differentialDrive.arcadeDrive(
            leftY.getAsDouble(),
            rightX.getAsDouble()
        );
      }
    }, this);
  }

  // Drives towards an april tag using 2D processing (3d solver). Stops a set distance away
  public Command driveToTarget(DoubleSupplier leftY, DoubleSupplier rightX, double desiredRange) {
    // Returns a command that runs periodically
    return Commands.run(() -> {
      // Get the latest camera results and target
      result = camera.getLatestResult();
      target = result.getBestTarget();

      // Only use vision information if there is a valid target
      if (result.hasTargets()) {
        // Calculates the distance to the target using constants along with the target's pitch
        // With only 1 variable, this is more stable than SolvePNP with a 6d solver (we can use this in 2D processing)
        double range = PhotonUtils.calculateDistanceToTargetMeters(
            PhotonVisionConstants.CAMERA_HEIGHT,
            PhotonVisionConstants.SPEAKER_TARGET_HEIGHT,
            PhotonVisionConstants.CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(target.getPitch())
        );
        // Calls arcadeDrive using the output of the drive PID controller and joystick input
        m_differentialDrive.arcadeDrive(
            driveController.calculate(range, desiredRange), //calculate a PID output using the targets's distance and the desired distance (meters)
            rightX.getAsDouble()
        );
        // Log information to SmartDashboard 
        SmartDashboard.putNumber("Distance To Target", range);
        SmartDashboard.putNumber("Desired Distance", desiredRange);
        SmartDashboard.putNumber("Vision Drive Speed", driveController.calculate(range, desiredRange));
      }
      // If there are no valid targets, call arcadeDrive with joystick input
      else {
        m_differentialDrive.arcadeDrive(
            leftY.getAsDouble(),
            rightX.getAsDouble()
        );
      }
    }, this);
  }

  // Drives and rotates towards an april tag using 2D processing (3d solver). Stops a set distance away
  public Command driveRotateToTarget(DoubleSupplier leftY, DoubleSupplier rightX, double desiredRange) {
    // Returns a command that runs periodically
    return Commands.run(() -> {
      // Get the latest camera results and target
      result = camera.getLatestResult();
      target = result.getBestTarget();

      // Only use vision information if there is a valid target
      if (result.hasTargets()) {
        // Calculates the distance to the target using constants along with the target's pitch
        // With only 1 variable, this is more stable than SolvePNP with a 6d solver (we can use this in 2D processing)
        double range = PhotonUtils.calculateDistanceToTargetMeters(
            PhotonVisionConstants.CAMERA_HEIGHT,
            PhotonVisionConstants.SPEAKER_TARGET_HEIGHT,
            PhotonVisionConstants.CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(target.getPitch())
        );
        // Calls arcadeDrive using the output of PID controllers
        m_differentialDrive.arcadeDrive(
            driveController.calculate(range, desiredRange), //calculate a PID output using the targets's distance and the desired distance (meters)
            -rotController.calculate(target.getYaw(), 0) //calcuate a PID output based on the target's yaw (in degrees)
        );
        // Log information to SmartDashboard 
        SmartDashboard.putNumber("Distance To Target", range);
        SmartDashboard.putNumber("Desired Distance", desiredRange);
        SmartDashboard.putNumber("Vision Drive Speed", driveController.calculate(range, desiredRange));
        SmartDashboard.putNumber("Rot Offset", target.getYaw());
        SmartDashboard.putNumber("Vision Rot Speed", -rotController.calculate(target.getYaw(), 0));
      }
      // If there are no valid targets, call arcadeDrive with joystick input
      else {
        m_differentialDrive.arcadeDrive(
            leftY.getAsDouble(),
            rightX.getAsDouble()
        );
      }
    }, this);
  }

  // Add a vision measurement to the existing WPILib PoseEstimator odometry
  // "timestampSeconds" is the timestamp associated with the visionPose2d passed as an argument
  // TIP: For the current timestamp, use "Timer.getFPGATimestamp()" (don't do that in this method)
  public void addVisionPose2d(Pose2d visionPose2d, double timestampSeconds) {
    m_driveOdometry.addVisionMeasurement(visionPose2d, timestampSeconds);
  }

  @Override
  public void periodic() {
    // Update drive odometry using encoders (without vision)
    // This is still important because addVisionMeasurement() is based on existing odometry
    m_driveOdometry.updateWithTime(
        Timer.getFPGATimestamp(),
        new Rotation2d(Units.degreesToRadians(gyro.getAngle())),
        -leftMotorEncoder.getPosition(),
        -rightMotorEncoder.getPosition()
    );

    // Get the latest camera results
    result = camera.getLatestResult();

    // Try to update "latestRobotPose" with a new "EstimatedRobotPose" using a "PhotonPoseEstimator"
    // If "latestRobotPose" is updated, call addVisionPose2d() and pass the updated "latestRobotPose" as an argument
    try {
      latestRobotPose = poseEstimator.update(result).get();
      addVisionPose2d(latestRobotPose.estimatedPose.toPose2d(), latestRobotPose.timestampSeconds);
    } catch (Exception e) { // catch = catching an exception, java.util.Optional.get() throws NoSuchElementException if no value is present
      latestRobotPose = null; // If there is no updated "EstimatedRobotPose", update "latestRobotPose" to null
    }
    
    // Log information to SmartDashboard 
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
