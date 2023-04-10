package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The intake subsysatem will be used to set up the motors and encoders for the
 * intake.
 */
public class Intake extends SubsystemBase {

    // Constants
    private static final double k_intakePercentage = .25;
    private static final double k_openLoopRampRate = 0.1;
    private static final int k_currentLimit = 39; // Current limit for intake falcon 500

    // Components
    private TalonFX m_intakeMotor;

    // private final CANSparkMax intakeMotor;
  //  private final double m_intakeEncoder;

    /* Game Piece Currently In Robot */

    /**
     * Constructor for intake subsystem.
     */
    public Intake() {
        m_intakeMotor = new TalonFX(Constants.Intake.motorId);// new CANSparkMax(Constants.Intake.motorId,
                                                                      // MotorType.kBrushless);
        // Configure all settings on Talons
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.voltageCompSaturation = 12.0;
        config.openloopRamp = k_openLoopRampRate;
        config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, k_currentLimit, 0, 0);

        m_intakeMotor.configAllSettings(config);
        m_intakeMotor.enableVoltageCompensation(true);
        m_intakeMotor.setNeutralMode(NeutralMode.Brake);
        m_intakeMotor.setInverted(TalonFXInvertType.Clockwise);
        m_intakeMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        // m_intakeEncoder = m_intakeMotor.getSelectedSensorPosition();// Encoder();

        //m_intakeMotor.setSmartCurrentLimit(Constants.Intake.currentLimit);

    }
    /*
     * public void setGamePiece(GamePiece piece) {
     * gamePiece = piece;
     * }
     * 
     * public GamePiece getGamePiece() {
     * return gamePiece;
     * }
     */

    public void setMotor(double speed) {
        m_intakeMotor.set(TalonFXControlMode.PercentOutput, 12/speed*k_intakePercentage); // setVoltage(speed);
    }

    public double getPDMCurrent() {
        return m_intakeMotor.getStatorCurrent();
    }

    public double getVelocity() {
        return m_intakeMotor.getSelectedSensorVelocity();
    }

    @Override
    public void periodic() {
        // returns in amps
        // double intakeCurrent = pdm.getCurrent(Constants.IntakeConstants.pdpChannel);
        double intakeCurrent = m_intakeMotor.getStatorCurrent();
        SmartDashboard.putNumber("Intake Current", intakeCurrent);
        SmartDashboard.putNumber("Intake Velocity", getVelocity());
        // SmartDashboard.putNumber("Gamepiece", getGamePiece().getDirection());

    }

    public void resetIntakeEncoder() {
        m_intakeMotor.setSelectedSensorPosition(0);
       // m_intakeEncoder= m_intakeMotor.getSelectedSensorPosition();

    }

}