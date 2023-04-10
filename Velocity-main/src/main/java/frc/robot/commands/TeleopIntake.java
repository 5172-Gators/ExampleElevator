package frc.robot.commands;

import java.util.function.BooleanSupplier;

//import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.Wrist.PIDFFmode;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class TeleopIntake extends CommandBase {
    private Intake s_Intake;
    private Wrist s_Wrist;
    private boolean moveVal;

    public TeleopIntake(Intake s_Intake, Wrist s_Wrist, BooleanSupplier moveVal) {
        this.s_Intake = s_Intake;
        this.s_Wrist = s_Wrist;
        this.moveVal = moveVal.getAsBoolean();

        addRequirements(s_Intake);
    }

    @Override
    public void execute() {
        double maxSpeed = RobotContainer.gamePiece == GamePiece.CONE ? Constants.Intake.coneIntakeSpeed
                : Constants.Intake.cubeIntakeSpeed;
        
        //TWP_ Xbox triggers return an analog. changed moveVal to boolean to work with our joystick trigger
                double power = 0;
        if (moveVal) {
            power = MathUtil.clamp(
                    ( maxSpeed + .5) * RobotContainer.gamePiece.getDirection(),
                    -maxSpeed,
                    maxSpeed);
        } else {
            power = 0;

        }


        s_Intake.setMotor(power);

        // Check if a cone was intaked, if so switch PID on wrist.
        if (power != 0 &&
                Math.abs(s_Intake.getVelocity()) < Constants.Intake.stoppedRPMThreshold
                && RobotContainer.gamePiece == GamePiece.CONE) {
            s_Wrist.setPIDFFMode(PIDFFmode.WEIGHTED);
        }
    }
}
