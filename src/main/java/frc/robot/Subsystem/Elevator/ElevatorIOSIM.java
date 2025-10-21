package frc.robot.Subsystem.Elevator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Constants.Constants;

public class ElevatorIOSIM extends ElevatorIOCTRE{
    private final TalonFXSimState elevatorRightSimState;
    private final TalonFXSimState elevatorLeftSimState;

    private final DCMotor elevatorMotorSim = DCMotor.getKrakenX60(2);

    private final ElevatorSim elevatorSim = new ElevatorSim(
        elevatorMotorSim,
        Constants.ElevatorConstants.ELEVATOR_GEAR_RATIO,
        Constants.ElevatorConstants.ELEVATOR_CARRIAGE_MASS,
        0.06,
        Constants.ElevatorConstants.ELEVATOR_MIN_LENGTH,
        Constants.ElevatorConstants.ELEVATOR_MAX_LENGTH,
        false,
        0.0
    );

    private final LoggedMechanism2d m_mech2d = new LoggedMechanism2d(2, 2);
    private final LoggedMechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 1, 0.5);
    private final LoggedMechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new LoggedMechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 90));

    private Pose3d elevatorStageOnePose = new Pose3d();
    private Pose3d elevatorStageTwoPose = new Pose3d();



    public ElevatorIOSIM(TalonFXConfiguration leftConfig, TalonFXConfiguration rightConfig){
        super(leftConfig, rightConfig);

        elevatorRightSimState = new TalonFXSimState(getElevatorRight());
        elevatorLeftSimState = new TalonFXSimState(getElevatorLeft());
    }

    public void update(){
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        super.update();

        elevatorLeftSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        elevatorSim.setInput(elevatorLeftSimState.getMotorVoltage());

        
        // Next, we update it. The standard loop time is 20ms.
        elevatorSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        elevatorLeftSimState.setRawRotorPosition(elevatorSim.getPositionMeters()/Constants.ElevatorConstants.ROTOR_TO_METERS*9);
        elevatorLeftSimState.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond()/Constants.ElevatorConstants.ROTOR_TO_METERS*9);
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

        updateTelemetry();
        Logger.recordOutput("Elevator/ElevatorSimMech2d", m_mech2d);
        Logger.recordOutput("Elevator/ElevatorStageOnePose3d", elevatorStageOnePose);
        Logger.recordOutput("Elevator/ElevatorStageTwoPose3d", elevatorStageTwoPose);
        Logger.recordOutput("Elevator/ElevatorSimPositionMeters", elevatorSim.getPositionMeters());


    }

    public void updateTelemetry() {
        // Update elevator visualization with position
        m_elevatorMech2d.setLength(getElevatorLeft().getPosition().getValueAsDouble()*Constants.ElevatorConstants.ROTOR_TO_METERS);
        elevatorStageOnePose = new Pose3d(0, 0, MathUtil.clamp(elevatorSim.getPositionMeters(),0,0.68), new Rotation3d( 0, 0, 0));
        elevatorStageTwoPose = new Pose3d(0, 0, elevatorSim.getPositionMeters(), new Rotation3d( 0, 0, 0));
    }

}
