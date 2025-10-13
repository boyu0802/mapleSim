package frc.robot.Subsystem.AlgaeIntake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.Constants;

public class IntakeIOSIM extends IntakeIOREV {
    
    private final DCMotor pivotGearbox = DCMotor.getNeoVortex(1);
    private final DCMotor rollerGearbox = DCMotor.getNeoVortex(1);

    private SparkFlexSim pivotSim;
    private SparkFlexSim rollerSim;
    
    public static final double kArmMass = 8.0; // Kilograms
    public static final double kArmLength = Units.inchesToMeters(30);

    private SingleJointedArmSim pivotArmSim = new SingleJointedArmSim(
        pivotGearbox,
        Constants.AlgaeIntakeConstants.PIVOT_MOTOR_GEAR_RATIO,
        SingleJointedArmSim.estimateMOI(kArmLength, kArmMass),
        kArmLength,
        Units.degreesToRadians(-140),
        Units.degreesToRadians(100),
        false,
        Units.degreesToRadians(-70)
    );

    @AutoLogOutput
    private Pose3d pivotArmPose = new Pose3d(-0.299, 0.3, 0.248, new Rotation3d(0,0,0));
// Units.inchesToMeters(24.6)
    private LoggedMechanism2d pivotArmMech = new LoggedMechanism2d(2, 2);
    private LoggedMechanismRoot2d pivotRoot = pivotArmMech.getRoot("pivotRoot",0.6,0.3);
    private LoggedMechanismLigament2d pivotTower =
        pivotRoot.append(new LoggedMechanismLigament2d("pivotTower", 0.110, -90, 2, new Color8Bit(Color.kBlue)));
    private LoggedMechanismLigament2d pivotArm =
        pivotRoot.append(new LoggedMechanismLigament2d("pivotArm",0.16, 0,2, new Color8Bit(Color.kBlue)));
    private LoggedMechanismLigament2d pivotArmA = 
        pivotArm.append(new LoggedMechanismLigament2d("pivotArmA", 0.142, 34 ,2, new Color8Bit(Color.kBlue)));
    private LoggedMechanismLigament2d pivotArmB =
        pivotArmA.append(new LoggedMechanismLigament2d("pivotArmB", 0.25, 40,2, new Color8Bit(Color.kBlue)));

    public IntakeIOSIM(SparkFlexConfig pivotConfig, SparkFlexConfig rollerConfig){
        super(pivotConfig, rollerConfig);


        pivotSim = new SparkFlexSim(getPivotFlex(), pivotGearbox);
        rollerSim = new SparkFlexSim(getRollerFlex(), rollerGearbox);


        pivotTower.setColor(new Color8Bit(Color.kAquamarine));
        pivotArm.setColor(new Color8Bit(Color.kLime));
        pivotArmA.setColor(new Color8Bit(Color.kNavajoWhite));
        pivotArmB.setColor(new Color8Bit(Color.kMediumVioletRed));

        pivotSim.setPosition(0);
    }

    public void update(){
        pivotArmSim.setInputVoltage(pivotSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        pivotArmSim.update(0.02);

        pivotSim.iterate(
            Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
                pivotArmSim.getVelocityRadPerSec())*Constants.AlgaeIntakeConstants.PIVOT_MOTOR_GEAR_RATIO/60,
            RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
            0.02);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(pivotArmSim.getCurrentDrawAmps()));

        pivotArm.setAngle(Units.radiansToDegrees(pivotArmSim.getAngleRads()));
        pivotArmPose = new Pose3d(-0.299, 0.3, 0.248, new Rotation3d(0,-pivotArmSim.getAngleRads(),0));
        
        Logger.recordOutput("arm_sim", pivotArmMech);
        Logger.recordOutput("intake/pivot/angle", Units.radiansToRotations(pivotArmSim.getAngleRads()));
        Logger.recordOutput("intake/pivot/velocity", pivotArmSim.getVelocityRadPerSec());
        Logger.recordOutput("intake/pivot/voltage", pivotArmSim.getInput().get(0, 0));
        super.periodic();
    }











}
