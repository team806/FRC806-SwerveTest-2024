package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenixpro.hardware.CANcoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class swervedrive extends drivetrain{

    private static final double attainableMaxModuleSpeedMetersPerSecond = 0;
    private static final double attainableMaxTranslationalSpeedMetersPerSecond = 0;
    private static final double attainableMaxRotationalVelocityRadiansPerSecond = 0;
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    //odometry.update(new Rotation2d(Math.toRadians(gyro.getAngle())), getPositions());
    swerveModule[] modules = {new swerveModule(),new swerveModule(),new swerveModule(),new swerveModule()};
    ADIS16470_IMU gyro;    

    public swervedrive(Translation2d[] moduleLocations, Spark[] steerMotors, TalonFX[] driveMotors, CANcoder[] encoders, ADIS16470_IMU gyro){

        kinematics = new SwerveDriveKinematics(moduleLocations);
        this.gyro = gyro;
        modules[0] = new swerveModule(steerMotors[0], driveMotors[0], encoders[0]);
        modules[1] = new swerveModule(steerMotors[1], driveMotors[1], encoders[1]);
        modules[2] = new swerveModule(steerMotors[2], driveMotors[2], encoders[2]);
        modules[3] = new swerveModule(steerMotors[3], driveMotors[3], encoders[3]);

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

        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        modulePositions[0] = modules[0].getPosition();
        modulePositions[1] = modules[1].getPosition();
        modulePositions[2] = modules[2].getPosition();
        modulePositions[3] = modules[3].getPosition();
        return modulePositions;

    }

}
