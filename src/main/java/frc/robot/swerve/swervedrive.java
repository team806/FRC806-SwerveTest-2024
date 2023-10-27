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
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class swervedrive extends drivetrain{
    
    SwerveDriveKinematics kinematics;
    swerveModule[] modules;
    swerveModule frontRight;
    swerveModule backLeft;
    swerveModule backRight; 
    startPose = new Pose2d(5.0, 13.5, new Rotation2d());

    swervedrive(
        Translation2d[] moduleLocations, Spark[] steerMotors, TalonFX[] driveMotors, CANcoder[] encoders){

            kinematics = new SwerveDriveKinematics(moduleLocations[0],moduleLocations[1],moduleLocations[2],moduleLocations[3]);

            SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
                kinematics, gyro.getRotation2d(), moduleLocations, startPose
            );

            modules = {
                new swerveModule(steerMotors[0], driveMotors[0], encoders[0]),
                new swerveModule(steerMotors[1], driveMotors[1], encoders[1]),
                new swerveModule(steerMotors[2], driveMotors[2], encoders[2]),
                new swerveModule(steerMotors[3], driveMotors[3], encoders[3])
            };
        }

        public void setVelocity(ChassisSpeeds speeds){
            SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
            kinematics.desaturateWheelSpeeds(moduleStates, speeds, 0, 0, 0);
            frontLeft.setState(moduleStates[0]);
            frontRight.setState(moduleStates[1]);
            backLeft.setState(moduleStates[2]);
            backRight.setState(moduleStates[3]);
        }
    

}
