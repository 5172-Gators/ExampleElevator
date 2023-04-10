package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Constants.Position;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.Constants.Position;

public class Elevator extends SubsystemBase {

    private final TalonFX elevatorMotorOne;
    private final TalonFX elevatorMotorTwo;
    final int kUnitsPerRevolution = 2048; /* this is constant for Talon FX */

    private static final double k_openLoopRampRate = 0.1;
    private static final int k_currentLimit = Constants.Elevator.currentLimit; // Current limit for intake falcon 500



    private PIDController pidController;

    private double currentPosition;

    /**
     * Initialize Elevator motor and the built in encoder. There are no cancoders on
     * the elevator
     */
    public Elevator() {
        // initialize motors
        // the right motor will spin clockwise and the left motor will go counter
        // clockwise
        elevatorMotorOne = new TalonFX(Constants.Elevator.motorOneId);
        elevatorMotorTwo = new TalonFX(Constants.Elevator.motorTwoId);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.voltageCompSaturation = 12.0;
        config.openloopRamp = k_openLoopRampRate;
        config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, k_currentLimit, 0, 0);

        elevatorMotorOne.configAllSettings(config);
        elevatorMotorOne.enableVoltageCompensation(true);
        elevatorMotorOne.setNeutralMode(NeutralMode.Brake);
        elevatorMotorOne.setInverted(TalonFXInvertType.Clockwise);
        elevatorMotorOne.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        elevatorMotorOne.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
        //elevatorMotorOne.setSelectedSensorPosition(0); // zero the encoder

        elevatorMotorTwo.configAllSettings(config);
        elevatorMotorTwo.enableVoltageCompensation(true);
        elevatorMotorTwo.setNeutralMode(NeutralMode.Brake);
        elevatorMotorTwo.setInverted(TalonFXInvertType.Clockwise);
        elevatorMotorTwo.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);






       // elevatorRightController = new CANCoder(Constants.Elevator.canConderRightId);

        // The motors will follow each other
        // The right motor will follow whatever the applied output on the
        // left motor is so only need to adjust output for the left motor

        // initialize pidContoller
        pidController = new PIDController(Constants.Elevator.elevatorKP, Constants.Elevator.elevatorKI,
                Constants.Elevator.elevatorKD);
        pidController.setSetpoint(0);
        pidController.setTolerance(.25);

        setPosition(Position.STANDBY.getElev());
    }

    public void resetEncoder() {
        elevatorMotorOne.setSelectedSensorPosition(0);// .getEncoder().setPosition(0);
        elevatorMotorTwo.setSelectedSensorPosition(0); //getEncoder().setPosition(0);
    }

    public Command setPositionCMD(double position) {
        return run(() -> setPosition(position)).until(() -> atSetpoint());
    }

    public void setPosition(double position) {
        if(position > 35) {
            position = 35;
        }
        else if(position < 0.1) {
            position = 0.1;
        }
        currentPosition = position;
    }

    public double getPosition() {
        return currentPosition;
    }

    public void move(double voltage) {
        elevatorMotorOne.set(ControlMode.PercentOutput, voltage/12);
        elevatorMotorTwo.set(ControlMode.PercentOutput, voltage/12);
    }

    public boolean reachedSetpoint(double distance) {
        return pidController.getPositionTolerance() >= Math.abs(currentPosition - distance);
    }

    public double getEncoderPosition() {
        return (elevatorMotorOne.getSelectedSensorPosition() + elevatorMotorTwo.getSelectedSensorPosition()) / 2;
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Elevator at setpoint", atSetpoint());
        SmartDashboard.putNumber("Elevator Position", getEncoderPosition());
        SmartDashboard.putNumber("Elevator Goal Position", currentPosition);

        move(
                MathUtil.clamp(
                        pidController.calculate(getEncoderPosition(), currentPosition),
                        -Constants.Elevator.maxMotorVoltage,
                        Constants.Elevator.maxMotorVoltage));
    }
}