package frc.robot.Constants;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public class DrivetrainConstants {
// -------------------------------- CAN IDs ----------------------------------------------------
    public static final String CANBUS_NAME = "rio";
    
    public static final int FRONT_LEFT_DRIVE_ID = 4;
    public static final int FRONT_LEFT_TURN_ID = 3;
    public static final int FRONT_LEFT_CANCODER_ID = 10;

    public static final int FRONT_RIGHT_DRIVE_ID = 2;
    public static final int FRONT_RIGHT_TURN_ID = 1;
    public static final int FRONT_RIGHT_CANCODER_ID = 9;

    public static final int BACK_LEFT_DRIVE_ID = 6;
    public static final int BACK_LEFT_TURN_ID = 5;
    public static final int BACK_LEFT_CANCODER_ID = 11;

    public static final int BACK_RIGHT_DRIVE_ID = 8;
    public static final int BACK_RIGHT_TURN_ID = 7;
    public static final int BACK_RIGHT_CANCODER_ID = 12;

    public static final int PIGEON_ID = 1;
// --------------------------------------------------------------------------------------------


// -------------------------------- GEAR RATIOS & DIMENSIONS & Limits --------------------------------    
    public static final double DRIVE_GEAR_RATIO = 4.5; //TODO: real values
    public static final double TURN_GEAR_RATIO = 20.0;
    public static final double COUPLING_GEAR_RATIO = 0.0;

    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.96);
    public static final double WHEEL_CIRCUMFERENCE_METERS = 2 * Math.PI * WHEEL_RADIUS_METERS;
    public static final double WHEEL_COF = 1.2;

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(24.6);
    public static final double DRIVETRAIN_TRACKLENGTH_METERS = Units.inchesToMeters(24.6);
    
    public static final double MAX_SPEED_METERS_PER_SECOND = 6.95; 
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_SPEED_METERS_PER_SECOND/
        Math.hypot(DRIVETRAIN_TRACKLENGTH_METERS/2.0, DRIVETRAIN_TRACKWIDTH_METERS/2.0);

        
    public static final double SLIPCURRENT = 120;

    public static final Translation2d[] MODULE_POSITIONS = new Translation2d[] {
        new Translation2d( DRIVETRAIN_TRACKLENGTH_METERS / 2.0,  DRIVETRAIN_TRACKWIDTH_METERS / 2.0), // Front Left
        new Translation2d(-DRIVETRAIN_TRACKLENGTH_METERS / 2.0,  DRIVETRAIN_TRACKWIDTH_METERS / 2.0), // Back Left
        new Translation2d(-DRIVETRAIN_TRACKLENGTH_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0), // Back Right
        new Translation2d( DRIVETRAIN_TRACKLENGTH_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0)  // Front Right
    };

// --------------------------------------------------------------------------------------------


//-------------------------------- Configs ---------------------------------------------------
    private static final Pigeon2Configuration PIGEON_CONFIGS = new Pigeon2Configuration();
    private static final TalonFXConfiguration initMotorConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(60)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true));

    

// -------------------------------- std deviations ------------------------------------------
    //TODO: tune these
    public static final Matrix<N3,N1> VISION_STD_DEVS = VecBuilder.fill(0.01,0.01,9999); // x, y, theta
    public static final Matrix<N3,N1> ODOMETRY_STD_DEVS = VecBuilder.fill(0.1,0.1,0.001); // x, y, theta

// -------------------------------------------------------------------------------------------------------------
    public static final SwerveDrivetrainConstants SWERVE_DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants()
        .withCANBusName(CANBUS_NAME)
        .withPigeon2Configs(PIGEON_CONFIGS)
        .withPigeon2Id(PIGEON_ID);

    public static final SwerveModuleConstants<TalonFXConfiguration,TalonFXConfiguration,CANcoderConfiguration> FRONT_LEFT_MODULE_CONSTANTS = new SwerveModuleConstants<TalonFXConfiguration,TalonFXConfiguration,CANcoderConfiguration>()
        .withDriveMotorId(FRONT_LEFT_DRIVE_ID)
        .withSteerMotorId(FRONT_LEFT_TURN_ID)
        .withEncoderId(FRONT_LEFT_CANCODER_ID)
        .withDriveMotorInitialConfigs(initMotorConfigs)
        .withSteerMotorInitialConfigs(initMotorConfigs)
        .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
        .withSteerMotorGearRatio(TURN_GEAR_RATIO)
        .withCouplingGearRatio(COUPLING_GEAR_RATIO)
        .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
        .withLocationX(MODULE_POSITIONS[0].getX())
        .withLocationY(MODULE_POSITIONS[0].getY())
        .withDriveMotorInverted(false) //TODO: check
        .withSteerMotorInverted(false) //TODO: check
        .withEncoderInverted(false) //TODO: check
        .withWheelRadius(WHEEL_RADIUS_METERS)
        .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
        .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
        .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
        .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
        .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
        .withDriveMotorGains(new Slot0Configs().withKP(0.1).withKV(0.124))
        .withSteerMotorGains(new Slot0Configs().withKP(100).withKD(0.5).withKV(1.91).withKS(0.1))
        .withDriveInertia(0.001)
        .withSteerInertia(0.00001)
        .withDriveFrictionVoltage(0.25)
        .withSteerFrictionVoltage(0.001);
    
    public static final SwerveModuleConstants<TalonFXConfiguration,TalonFXConfiguration,CANcoderConfiguration> BACK_LEFT_MODULE_CONSTANTS = new SwerveModuleConstants<TalonFXConfiguration,TalonFXConfiguration,CANcoderConfiguration>()
        .withDriveMotorId(BACK_LEFT_DRIVE_ID)
        .withSteerMotorId(BACK_LEFT_TURN_ID)
        .withEncoderId(BACK_LEFT_CANCODER_ID)
        .withDriveMotorInitialConfigs(initMotorConfigs)
        .withSteerMotorInitialConfigs(initMotorConfigs)
        .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
        .withSteerMotorGearRatio(TURN_GEAR_RATIO)
        .withCouplingGearRatio(COUPLING_GEAR_RATIO)
        .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
        .withLocationX(MODULE_POSITIONS[1].getX())
        .withLocationY(MODULE_POSITIONS[1].getY())
        .withDriveMotorInverted(false) //TODO: check
        .withSteerMotorInverted(false) //TODO: check
        .withEncoderInverted(false) //TODO: check
        .withWheelRadius(WHEEL_RADIUS_METERS)
        .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
        .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
        .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
        .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
        .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
        .withDriveMotorGains(new Slot0Configs().withKP(0.1).withKV(0.124))
        .withSteerMotorGains(new Slot0Configs().withKP(100).withKD(0.5).withKV(1.91).withKS(0.1));
    
    public static final SwerveModuleConstants<TalonFXConfiguration,TalonFXConfiguration,CANcoderConfiguration> BACK_RIGHT_MODULE_CONSTANTS = new SwerveModuleConstants<TalonFXConfiguration,TalonFXConfiguration,CANcoderConfiguration>()
        .withDriveMotorId(BACK_RIGHT_DRIVE_ID)
        .withSteerMotorId(BACK_RIGHT_TURN_ID)
        .withEncoderId(BACK_RIGHT_CANCODER_ID)
        .withDriveMotorInitialConfigs(initMotorConfigs)
        .withSteerMotorInitialConfigs(initMotorConfigs)
        .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
        .withSteerMotorGearRatio(TURN_GEAR_RATIO)
        .withCouplingGearRatio(COUPLING_GEAR_RATIO)
        .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
        .withLocationX(MODULE_POSITIONS[2].getX())
        .withLocationY(MODULE_POSITIONS[2].getY())
        .withDriveMotorInverted(false) //TODO: check
        .withSteerMotorInverted(false) //TODO: check
        .withEncoderInverted(false) //TODO: check
        .withWheelRadius(WHEEL_RADIUS_METERS)
        .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
        .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
        .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
        .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
        .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
        .withDriveMotorGains(new Slot0Configs().withKP(0.1).withKV(0.124))
        .withSteerMotorGains(new Slot0Configs().withKP(100).withKD(0.5).withKV(1.91).withKS(0.1));
    
    public static final SwerveModuleConstants<TalonFXConfiguration,TalonFXConfiguration,CANcoderConfiguration> FRONT_RIGHT_MODULE_CONSTANTS = new SwerveModuleConstants<TalonFXConfiguration,TalonFXConfiguration,CANcoderConfiguration>()
        .withDriveMotorId(FRONT_RIGHT_DRIVE_ID)
        .withSteerMotorId(FRONT_RIGHT_TURN_ID)
        .withEncoderId(FRONT_RIGHT_CANCODER_ID)
        .withDriveMotorInitialConfigs(initMotorConfigs)
        .withSteerMotorInitialConfigs(initMotorConfigs)
        .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
        .withSteerMotorGearRatio(TURN_GEAR_RATIO)
        .withCouplingGearRatio(COUPLING_GEAR_RATIO)
        .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
        .withLocationX(MODULE_POSITIONS[3].getX())
        .withLocationY(MODULE_POSITIONS[3].getY())
        .withDriveMotorInverted(false) //TODO: check
        .withSteerMotorInverted(false) //TODO: check
        .withEncoderInverted(false) //TODO: check
        .withWheelRadius(WHEEL_RADIUS_METERS)
        .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
        .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
        .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
        .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
        .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
        .withDriveMotorGains(new Slot0Configs().withKP(0.1).withKV(0.124))
        .withSteerMotorGains(new Slot0Configs().withKP(100).withKD(0.5).withKV(1.91).withKS(0.1));

    @SuppressWarnings("unchecked")
    public static final SwerveModuleConstants<TalonFXConfiguration,TalonFXConfiguration,CANcoderConfiguration>[] MODULE_CONSTANTS = new SwerveModuleConstants[]{
        FRONT_LEFT_MODULE_CONSTANTS,
        BACK_LEFT_MODULE_CONSTANTS,
        BACK_RIGHT_MODULE_CONSTANTS,
        FRONT_RIGHT_MODULE_CONSTANTS
    };

    public static final DriveTrainSimulationConfig DRIVETRAIN_SIMULATED_CONFIGS = DriveTrainSimulationConfig.Default()
        .withBumperSize(Meters.of(0.9),Meters.of(0.9))
        .withRobotMass(Kilograms.of(63))
        .withTrackLengthTrackWidth(Meters.of(DrivetrainConstants.DRIVETRAIN_TRACKLENGTH_METERS),Meters.of(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS))
        .withSwerveModule(new SwerveModuleSimulationConfig(
            DCMotor.getFalcon500(1),
            DCMotor.getFalcon500(1),
            4.8,
            20.0,
            Volts.of(0.1),
            Volts.of(0.15),
            Meters.of(WHEEL_RADIUS_METERS),
            KilogramSquareMeters.of(0.35),
            WHEEL_COF
        ));

    
}
