package frc.robot.swerve;

import com.ctre.phoenixpro.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;



public class swerveModule {

private static final double driveGearRatio = 0;

CANSparkMax steer;
RelativeEncoder steerEncoder;
CANSparkMax drive;
RelativeEncoder driveEncoder;
CANcoder encoder;
PIDController steerController;
PIDController driveController;

    swerveModule(CANSparkMax Steer, CANSparkMax Drive, CANcoder encoder){
        this.steer = Steer;
        steerEncoder = Steer.getEncoder();
        this.drive = Drive;
        Drive.getEncoder();
        this.encoder = encoder;
        steerController = new PIDController(0.001, 0, 0, 0.02);
        steerController.enableContinuousInput(0, 1);
        driveController = new PIDController(0.001, 0, 0, 0.02);
    }

    public void setState(SwerveModuleState swerveModuleState) {
        steer.set(steerController.calculate(encoder.getAbsolutePosition().getValue(), swerveModuleState.angle.getRotations()));//FIXME change to set voltage
        drive.set(steerController.calculate(driveEncoder.getVelocity() * driveGearRatio, swerveModuleState.speedMetersPerSecond));//FIXME change to set voltage
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(0,new Rotation2d(Math.toRadians(encoder.getAbsolutePosition().getValue())));//FIXME i dont even know what this is ment to do
    }
}
