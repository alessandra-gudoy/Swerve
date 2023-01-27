package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {

        // the maximum voltage that will be delivered to the drive motors.
        public static final double MAX_VOLTAGE = 9.0;

        // robot's max speed (m/s) ** how fast robot drives in straight line
        public static final double MAX_VELOCITY_METERS_PER_SECOND = //13.5 / 60.0 * SdsModuleConfigurations.MK4I_L1.getDriveReduction() * SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;
                         6380.0 / 60.0 *
                        SdsModuleConfigurations.MK4I_L1.getDriveReduction() *
                        SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;

        // maximum angular velocity (rad/sec) ** how fast robot rotates
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Front right
                        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back left
                        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back right
                        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

        // gyroscope
        // private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
        // private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX
        // connected over MXP
        private final AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 200);

        // swerve modules ** initialized in constructor
        private final SwerveModule m_frontLeftModule;
        private final SwerveModule m_frontRightModule;
        private final SwerveModule m_backLeftModule;
        private final SwerveModule m_backRightModule;

        private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        public DrivetrainSubsystem() {
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

                // swerve helpers
                // Mk3SwerveModuleHelper.createFalcon500(...) (two falcons)
                // Mk3SwerveModuleHelper.createNeo(...) (two neos)
                // Mk3SwerveModuleHelper.createFalcon500Neo(...) (falcon500=driving,
                // neo=turning)
                // Mk3SwerveModuleHelper.createNeoFalcon500(...) (neo=driving,
                // falcon500=turning)

                // Mk4SwerveModuleHelper.createFalcon500Neo(gearRatio, driveMotorId,
                // turnMotorId, turnEncoderId, turnEncoderOffset);
                // Before gear ratio (see current state of module on dashboard)
                /*
                 * tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                 * .withSize(2, 4)
                 * .withPosition(0, 0)
                 */
                
                m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
                                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                               .withSize(2, 4).withPosition(0, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L1,
                                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                                FRONT_LEFT_MODULE_STEER_MOTOR,
                                FRONT_LEFT_MODULE_STEER_ENCODER,
                                FRONT_LEFT_MODULE_STEER_OFFSET);

                m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
                                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                               .withSize(2, 4).withPosition(0, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L1,
                                BACK_LEFT_MODULE_DRIVE_MOTOR,
                                BACK_LEFT_MODULE_STEER_MOTOR,
                                BACK_LEFT_MODULE_STEER_ENCODER,
                                BACK_LEFT_MODULE_STEER_OFFSET);

                m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
                                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                               .withSize(2, 4).withPosition(0, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L1,
                                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_ENCODER,
                                FRONT_RIGHT_MODULE_STEER_OFFSET);

                m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
                                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                               .withSize(2, 4).withPosition(0, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L1,
                                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                BACK_RIGHT_MODULE_STEER_MOTOR,
                                BACK_RIGHT_MODULE_STEER_ENCODER,
                                BACK_RIGHT_MODULE_STEER_OFFSET);
        }

        // zeroes gyroscope ** used to set direction of robot to "forward"
        public void zeroGyroscope() {
                // m_pigeon.setFusedHeading(0.0);
                navx.zeroYaw();
        }

        public Rotation2d getGyroscopeRotation() {
                // return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());

                /*if (navx.isMagnetometerCalibrated()) {
                        // only get valid fused headings if the magnetometer is calibrated
                        return Rotation2d.fromDegrees(navx.getFusedHeading());
                }*/
                // rotating the robot counter-clockwise makes the angle increase.
                return Rotation2d.fromDegrees(360.0 - navx.getYaw());
                //return null;
        }

        public double getDegrees(){
                return navx.getCompassHeading();
        }

        public void drive(ChassisSpeeds chassisSpeeds) {
                m_chassisSpeeds = chassisSpeeds;
        }

        @Override
        public void periodic() {
                SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

                m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[0].angle.getRadians());
                m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[1].angle.getRadians());
                m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[2].angle.getRadians());
                m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[3].angle.getRadians());

                SmartDashboard.putNumber("BACK LEFT: ", m_backLeftModule.getSteerAngle());
                SmartDashboard.putNumber("BACK RIGHT: ", m_backRightModule.getSteerAngle());
                SmartDashboard.putNumber("FRONT LEFT: ", m_frontLeftModule.getSteerAngle());
                SmartDashboard.putNumber("FRONT RIGHT: ", m_frontRightModule.getSteerAngle());
                SmartDashboard.putNumber("NAVX: YAW", navx.getYaw());
                SmartDashboard.putNumber("NAVX l:", navx.getCompassHeading());
                SmartDashboard.putString("Auto: ", "Periodic method");

                SmartDashboard.putNumber("Drive Velocity", m_frontLeftModule.getDriveVelocity());
        }
}
