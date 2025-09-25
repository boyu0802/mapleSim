package frc.robot.Constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public class SimDrivetrainConstant {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains =
            new Slot0Configs()
                    .withKP(100)
                    .withKI(0)
                    .withKD(0.5)
                    .withKS(0.1)
                    .withKV(0.0)
                    .withKA(0)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains =
            new Slot0Configs().withKP(10.0).withKI(0.0).withKD(0.0).withKS(1.5).withKV(0.0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType kDriveClosedLoopOutput =
            ClosedLoopOutputType.TorqueCurrentFOC;

    // The type of motor used for the drive motor
    private static final DriveMotorArrangement kDriveMotorType =
            DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the drive motor
    private static final SteerMotorArrangement kSteerMotorType =
            SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current kSlipCurrent = Amps.of(120.0);

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private static final TalonFXConfiguration driveInitialConfigs =
            new TalonFXConfiguration()
                    .withMotorOutput(
                            new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    private static final TalonFXConfiguration steerInitialConfigs =
            new TalonFXConfiguration()
                    .withCurrentLimits(
                            new CurrentLimitsConfigs()
                                    // Swerve azimuth does not require much torque output, so we can
                                    // set a relatively
                                    // low
                                    // stator current limit to help avoid brownouts without
                                    // impacting performance.
                                    .withSupplyCurrentLimit(Amps.of(60))
                                    .withSupplyCurrentLimitEnable(true))
                    .withMotorOutput(
                            new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus kCANBus = new CANBus("rio");

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(6.95);

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 0.0;

    private static final double kDriveGearRatio = 4.5;            ;
    private static final double kSteerGearRatio = 20;
    private static final Distance kWheelRadius = Inches.of(1.96);

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final int kPigeonId = 1;

    // These are only used for simulation
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
    // Simulated voltage necessary to overcome friction
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants =
            new SwerveDrivetrainConstants()
                    .withCANBusName(kCANBus.getName())
                    .withPigeon2Id(kPigeonId)
                    .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            ConstantCreator =
                    new SwerveModuleConstantsFactory<
                                    TalonFXConfiguration,
                                    TalonFXConfiguration,
                                    CANcoderConfiguration>()
                            .withDriveMotorGearRatio(kDriveGearRatio)
                            .withSteerMotorGearRatio(kSteerGearRatio)
                            .withCouplingGearRatio(kCoupleRatio)
                            .withWheelRadius(kWheelRadius)
                            .withSteerMotorGains(steerGains)
                            .withDriveMotorGains(driveGains)
                            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                            .withSlipCurrent(kSlipCurrent)
                            .withSpeedAt12Volts(kSpeedAt12Volts)
                            .withDriveMotorType(kDriveMotorType)
                            .withSteerMotorType(kSteerMotorType)
                            .withFeedbackSource(kSteerFeedbackType)
                            .withDriveMotorInitialConfigs(driveInitialConfigs)
                            .withSteerMotorInitialConfigs(steerInitialConfigs)
                            .withEncoderInitialConfigs(encoderInitialConfigs)
                            .withSteerInertia(kSteerInertia)
                            .withDriveInertia(kDriveInertia)
                            .withSteerFrictionVoltage(kSteerFrictionVoltage)
                            .withDriveFrictionVoltage(kDriveFrictionVoltage);

    // Front Left
    private static final int kFrontLeftDriveMotorId = 4;
    private static final int kFrontLeftSteerMotorId = 3;
    private static final int kFrontLeftEncoderId = 10;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.418701171875);
    private static final boolean kFrontLeftSteerMotorInverted = false;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(24.6);
    private static final Distance kFrontLeftYPos = Inches.of(24.6);

    // Front Right
    private static final int kFrontRightDriveMotorId = 2;
    private static final int kFrontRightSteerMotorId = 1;
    private static final int kFrontRightEncoderId = 9;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(0.420654296875);
    private static final boolean kFrontRightSteerMotorInverted = true;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(24.6);
    private static final Distance kFrontRightYPos = Inches.of(-24.6);

    // Back Left
    private static final int kBackLeftDriveMotorId = 6;
    private static final int kBackLeftSteerMotorId = 5;
    private static final int kBackLeftEncoderId = 11;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.433349609375);
    private static final boolean kBackLeftSteerMotorInverted = true;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-24.6);
    private static final Distance kBackLeftYPos = Inches.of(24.6);

    // Back Right
    private static final int kBackRightDriveMotorId = 8;
    private static final int kBackRightSteerMotorId = 7;
    private static final int kBackRightEncoderId = 12;
    private static final Angle kBackRightEncoderOffset = Rotations.of(0.20068359375);
    private static final boolean kBackRightSteerMotorInverted = true;
    private static final boolean kBackRightEncoderInverted = false;

    private static final Distance kBackRightXPos = Inches.of(-24.6);
    private static final Distance kBackRightYPos = Inches.of(-24.6);

    public static final SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            FrontLeft =
                    ConstantCreator.createModuleConstants(
                            kFrontLeftSteerMotorId,
                            kFrontLeftDriveMotorId,
                            kFrontLeftEncoderId,
                            kFrontLeftEncoderOffset,
                            kFrontLeftXPos,
                            kFrontLeftYPos,
                            kInvertLeftSide,
                            kFrontLeftSteerMotorInverted,
                            kFrontLeftEncoderInverted);
    public static final SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            FrontRight =
                    ConstantCreator.createModuleConstants(
                            kFrontRightSteerMotorId,
                            kFrontRightDriveMotorId,
                            kFrontRightEncoderId,
                            kFrontRightEncoderOffset,
                            kFrontRightXPos,
                            kFrontRightYPos,
                            kInvertRightSide,
                            kFrontRightSteerMotorInverted,
                            kFrontRightEncoderInverted);
    public static final SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            BackLeft =
                    ConstantCreator.createModuleConstants(
                            kBackLeftSteerMotorId,
                            kBackLeftDriveMotorId,
                            kBackLeftEncoderId,
                            kBackLeftEncoderOffset,
                            kBackLeftXPos,
                            kBackLeftYPos,
                            kInvertLeftSide,
                            kBackLeftSteerMotorInverted,
                            kBackLeftEncoderInverted);
    public static final SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            BackRight =
                    ConstantCreator.createModuleConstants(
                            kBackRightSteerMotorId,
                            kBackRightDriveMotorId,
                            kBackRightEncoderId,
                            kBackRightEncoderOffset,
                            kBackRightXPos,
                            kBackRightYPos,
                            kInvertRightSide,
                            kBackRightSteerMotorInverted,
                            kBackRightEncoderInverted);

    @SuppressWarnings("unchecked")
    public static final SwerveModuleConstants<TalonFXConfiguration,TalonFXConfiguration,CANcoderConfiguration>[] MODULE_CONSTANTS = new SwerveModuleConstants[]{
        FrontLeft, FrontRight, BackLeft, BackRight
    };
}
