package frc.robot.swerve;

import com.ctre.phoenixpro.hardware.CANcoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

public class swervedrive extends drivetrain{

    private static final double attainableMaxModuleSpeedMetersPerSecond = 0;//FIXME move to constants
    private static final double attainableMaxTranslationalSpeedMetersPerSecond = 0;
    private static final double attainableMaxRotationalVelocityRadiansPerSecond = 0;
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    swerveModule[] modules;
    ADIS16470_IMU gyro;    

    public swervedrive(Translation2d[] moduleLocations, CANSparkMax[] steerMotors, CANSparkMax[] driveMotors, CANcoder[] encoders, ADIS16470_IMU gyro, Pose2d startPose){

        modules[0] = new swerveModule(steerMotors[0], driveMotors[0], encoders[0]);//FIXME add configs in constructor
        modules[1] = new swerveModule(steerMotors[1], driveMotors[1], encoders[1]);
        modules[2] = new swerveModule(steerMotors[2], driveMotors[2], encoders[2]);
        modules[3] = new swerveModule(steerMotors[3], driveMotors[3], encoders[3]);
        this.gyro = gyro;
        kinematics = new SwerveDriveKinematics(moduleLocations);
        odometry = new SwerveDriveOdometry(kinematics, getGyroRotation2D(), getPositions(), startPose);

    }

    public void setVelocity(ChassisSpeeds speeds){

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, speeds, attainableMaxModuleSpeedMetersPerSecond, attainableMaxTranslationalSpeedMetersPerSecond , attainableMaxRotationalVelocityRadiansPerSecond );
        modules[0].setState(moduleStates[0]);
        modules[1].setState(moduleStates[1]);
        modules[2].setState(moduleStates[2]);
        modules[3].setState(moduleStates[3]);
        
    }

    public SwerveModulePosition[] getPositions(){

        return new SwerveModulePosition[] {modules[0].getPosition(), modules[1].getPosition(), modules[2].getPosition(), modules[3].getPosition()};

    }

    public void updateOdometry(){

        odometry.update(getGyroRotation2D(), getPositions());

    }

    public void resetOdometry(Pose2d pose){
        odometry.resetPosition(getGyroRotation2D(), getPositions(), pose);
    }

    public Pose2d getPose(){

        return odometry.getPoseMeters();

    }

    public void zeroGyro(){

        gyro.reset();

    }

    public Rotation2d getGyroRotation2D(){

        return new Rotation2d(Math.toRadians(gyro.getAngle()));
        
    }


}
