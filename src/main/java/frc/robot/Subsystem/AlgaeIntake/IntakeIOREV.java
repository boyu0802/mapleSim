package frc.robot.Subsystem.AlgaeIntake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Constants;

public class IntakeIOREV implements IntakeIO{
    private SparkFlex pivotFlex = new SparkFlex(5,MotorType.kBrushless);
    private SparkFlex rollerFlex = new SparkFlex(6,MotorType.kBrushless);

    private double setpoint = 0.0;

    public IntakeIOREV(SparkFlexConfig pivotConfig, SparkFlexConfig rollerConfig){
        
        pivotFlex.configure(pivotConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        rollerFlex.configure(rollerConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        
    }

    public void updateInputs(IntakeIOInputs inputs){
        inputs.pivotData = new IntakePivotData(
            Rotation2d.fromRotations(pivotFlex.getAbsoluteEncoder().getPosition()),
            pivotFlex.getEncoder().getPosition(),
            pivotFlex.getBusVoltage()*pivotFlex.getAppliedOutput(),
            pivotFlex.getEncoder().getVelocity()
        );

        inputs.rollerData = new IntakeRollerData(
            rollerFlex.getEncoder().getPosition(),
            rollerFlex.getBusVoltage()*rollerFlex.getAppliedOutput(),
            rollerFlex.getEncoder().getVelocity()
        );
    }

    public void runPivotVoltage(double volts){
        pivotFlex.setVoltage(volts);
    }

    public void runPivotPosition(double positionRotations){
        pivotFlex.getClosedLoopController().setReference(positionRotations, com.revrobotics.spark.SparkBase.ControlType.kPosition,ClosedLoopSlot.kSlot0,Constants.AlgaeIntakeConstants.PIVOT_FEED_FORWARD.calculate(Units.rotationsToRadians(positionRotations),0));
        setpoint = positionRotations;
    }

    public void runRollerVelocity(double velocityRotationsPerSecond){
        rollerFlex.getClosedLoopController().setReference(velocityRotationsPerSecond, com.revrobotics.spark.SparkBase.ControlType.kVelocity);
    }

    public SparkFlex getPivotFlex(){
        return pivotFlex;
    }

    public SparkFlex getRollerFlex(){
        return rollerFlex;
    }



    public void periodic(){
        Logger.recordOutput("intake/pivot/Motor Setpoint Rotation", setpoint);
        Logger.recordOutput("intake/pivot/Motor Absolute Encoder value Rotation", pivotFlex.getAbsoluteEncoder().getPosition());   
    }

}
