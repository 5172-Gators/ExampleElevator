// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Position;
import frc.robot.Constants.SEGMENT;
import frc.robot.Constants.Intake.EjectSpeed;
import frc.robot.Constants.LEDs.LEDMode;
import frc.robot.Constants.Wrist.PIDFFmode;
import frc.robot.autos.AutoBalancing;
import frc.robot.autos.segmentLineUp;
import frc.robot.Constants.GamePiece;
import frc.robot.commands.*;
import frc.robot.commands.TimedIntake.Direction;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LEDs.LEDs;

public class RobotContainer {
        /* Controllers */
        private final static Joystick translateStick = new Joystick(0);
        private final static Joystick rotateStick = new Joystick(1);
        private final static Joystick operatorStick = new Joystick(2);

        // private final CommandXboxController driver = new CommandXboxController(0);
        // private final CommandXboxController operator = new CommandXboxController(1);

        /* Drive Controls */
        private static final int translationAxis = Joystick.AxisType.kY.value;// XboxController.Axis.kLeftY.value;
        private static final int strafeAxis = Joystick.AxisType.kX.value;// XboxController.Axis.kLeftX.value;
        private static final int rotationAxis = Joystick.AxisType.kX.value;// XboxController.Axis.kRightX.value;
        // translateStick Buttons
        private static final JoystickButton driveFastTrigger = new JoystickButton(translateStick, 1);
        private static final JoystickButton driveSlowButton = new JoystickButton(translateStick, 2);
        private static final JoystickButton robotCentric = new JoystickButton(translateStick, 3);

        private static final JoystickButton gridLineUpButton = new JoystickButton(translateStick, 4);
        // rtotateStick Buttons
        private static final JoystickButton selectGamepieceTrigger = new JoystickButton(rotateStick, 1);
        private static final JoystickButton resetGyroButton = new JoystickButton(rotateStick, 2);
        private static final JoystickButton autoBalanceButton = new JoystickButton(rotateStick, 3);
        private static final JoystickButton followPathButton = new JoystickButton(rotateStick, 4);

        /* Operator Controls */
        private static final int elevatorAxis = Joystick.AxisType.kY.value;
        private static final int wristAxis = Joystick.AxisType.kX.value;

        private static final JoystickButton intakeTrigger = new JoystickButton(translateStick, 1);
        private static final JoystickButton outTakeSlowButton = new JoystickButton(operatorStick, 2);
        private static final JoystickButton outTakeFastButton = new JoystickButton(operatorStick, 3);
        private static final JoystickButton toggleLEDButton = new JoystickButton(operatorStick, 4);
        private static final JoystickButton pickHumanPlayerButton = new JoystickButton(operatorStick, 5);
        private static final JoystickButton pickStandingConeButton = new JoystickButton(operatorStick, 6);
        private static final JoystickButton pickTippedConeButton = new JoystickButton(operatorStick, 7);
        private static final JoystickButton pickCubeButton = new JoystickButton(operatorStick, 8);
        private static final JoystickButton stowIntakeButton = new JoystickButton(operatorStick, 9);
        private static final JoystickButton placeHighButton = new JoystickButton(operatorStick, 10);
        private static final JoystickButton placeMidButton = new JoystickButton(operatorStick,11);
        private static final JoystickButton placeLowButton = new JoystickButton(operatorStick, 12);


        /* Subsystems */
        private final Vision s_Vision = new Vision();
        private final Swerve s_Swerve = new Swerve(s_Vision);
        private final Intake s_Intake = new Intake();
        private final Elevator s_Elevator = new Elevator();
        private final Wrist s_Wrist = new Wrist();
        private final LEDs s_LEDs = new LEDs();

        public static GamePiece gamePiece = GamePiece.CONE;

        /* Autonomous Mode Chooser */
        private final SendableChooser<PathPlannerTrajectory> autoChooser = new SendableChooser<>();

        /* Autonomous */
        private final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                        s_Swerve::getPose,
                        s_Swerve::resetOdometry,
                        Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
                        new PIDConstants(Constants.Autonomous.kPXController, 0, 0),
                        new PIDConstants(Constants.Autonomous.kPThetaController,
                                        0,
                                        0),
                        s_Swerve::setModuleStates,
                        eventMap,
                        true,
                        s_Swerve);

        private static Map<String, Command> eventMap = new HashMap<>();
        {

                eventMap.put("setStandbyPosition",
                                new SetPosition(s_Wrist, s_Elevator, Position.STANDBY, () -> gamePiece));
                eventMap.put("setCone3Position",
                                new SequentialCommandGroup(
                                                s_Elevator.setPositionCMD(Position.CONEHIGH.getElev()),
                                                new WaitCommand(.5),
                                                s_Wrist.setPositionCMD(Position.CONEHIGH.getWrist() + .05),
                                                new WaitCommand(1)));
                eventMap.put("setCube3Position",
                                new SequentialCommandGroup(
                                                s_Wrist.setPositionCMD(Position.CUBEHIGH.getWrist()),
                                                s_Elevator.setPositionCMD(Position.CUBEHIGH.getElev()),
                                                new WaitCommand(1)));

                eventMap.put("setCubeIntakePosition",
                                new SetPosition(s_Wrist, s_Elevator, Position.CUBEINTAKE, () -> GamePiece.CUBE));
                eventMap.put("setStandingConeIntakePosition", new SetPosition(s_Wrist, s_Elevator,
                                Position.STANDINGCONEINTAKE, () -> GamePiece.CONE));

                eventMap.put("setTippedConeIntakePosition", new SetPosition(s_Wrist, s_Elevator,
                                Position.TIPPEDCONEINTAKE, () -> GamePiece.CONE));

                eventMap.put("coneDeposit",
                                new SequentialCommandGroup(
                                                new InstantCommand(() -> s_Wrist.setPIDFFMode(PIDFFmode.UNWEIGHTED)),
                                                new TimedIntake(s_Intake, .3, GamePiece.CONE, EjectSpeed.NORMAL,
                                                                Direction.OUTTAKE)));
                eventMap.put("cubeDeposit",
                                new TimedIntake(s_Intake, .3, GamePiece.CUBE, EjectSpeed.NORMAL, Direction.OUTTAKE));

                eventMap.put("runCubeIntake3",
                                new TimedIntake(s_Intake, 3, GamePiece.CONE, EjectSpeed.NORMAL, Direction.INTAKE));
                eventMap.put("runConeIntake3",
                                new SequentialCommandGroup(
                                                new InstantCommand(() -> s_Wrist.setPIDFFMode(PIDFFmode.WEIGHTED)),
                                                new TimedIntake(s_Intake, 3, GamePiece.CUBE, EjectSpeed.NORMAL,
                                                                Direction.INTAKE)));

                eventMap.put("wait1Seconds", new WaitCommand(1));

                eventMap.put("AutoBalance", new AutoBalancing(s_Swerve, true));
        }

        private final PathPlannerTrajectory moveForward = PathPlanner.loadPath("Move Forward",
                        Constants.Autonomous.kMaxSpeedMetersPerSecond,
                        Constants.Autonomous.kMaxAccelerationMetersPerSecondSquared);

        private final PathPlannerTrajectory sCurve = PathPlanner.loadPath("S Curve",
                        Constants.Autonomous.kMaxSpeedMetersPerSecond,
                        Constants.Autonomous.kMaxAccelerationMetersPerSecondSquared);

        private final PathPlannerTrajectory autoBalance = PathPlanner.loadPath("Autobalance",
                        1,
                        3);

        private final PathPlannerTrajectory backnForth = PathPlanner.loadPath("BacknForth",
                        Constants.Autonomous.kMaxSpeedMetersPerSecond,
                        Constants.Autonomous.kMaxAccelerationMetersPerSecondSquared);

        private final PathPlannerTrajectory coneCubeDeposit = PathPlanner.loadPath("coneCubeDeposit",
                        3,
                        1);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                CameraServer.startAutomaticCapture();

                // Sets each subsystem's default commands
                setDefaultCommands();

                // Configure the button bindings
                configureButtonBindings();

                // Configure Smart Dashboard options
                configureSmartDashboard();

                // Configure autonomous routines
                configureAutonomousPaths();
        }

        private void setDefaultCommands() {
                s_Swerve.setDefaultCommand(
                                new TeleopSwerve(
                                                s_Swerve,
                                                s_LEDs,
                                                () -> -translateStick.getRawAxis(translationAxis),
                                                () -> -translateStick.getRawAxis(strafeAxis),
                                                () -> -rotateStick.getRawAxis(rotationAxis),
                                                () -> robotCentric.getAsBoolean(), // translateStick.getRawButton(3);//
                                                                                   // button(t_Button3).getAsBoolean();//
                                                                                   // povDown().getAsBoolean(),
                                                () -> driveSlowButton.getAsBoolean(), // translateStick.getRawButton
                                                                                      // (4);//button(t_Button4).getAsBoolean();//leftBumper().getAsBoolean(),
                                                () -> driveFastTrigger.getAsBoolean(), // rotateStick.getRawButton
                                                                                       // (3);//JoystickButton(5);//rightBumper().getAsBoolean(),
                                                () -> gridLineUpButton.getAsBoolean()// rotateStick.getRawButton
                                                                                     // (4);//JoystickButton(6);//a().getAsBoolean()));
                                ));
                s_Elevator.setDefaultCommand(
                                new TeleopElevator(
                                                s_Elevator,
                                                () -> operatorStick.getRawAxis(elevatorAxis)));

                s_Wrist.setDefaultCommand(
                                new TeleopWrist(
                                                s_Wrist,
                                                () -> operatorStick.getRawAxis(wristAxis)));

                s_Intake.setDefaultCommand(
                                new TeleopIntake(s_Intake, s_Wrist,
                                                () -> intakeTrigger.getAsBoolean()));
        }

        private void configureAutonomousPaths() {

        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         * <p>
         * This method binds the buttons to commands.
         */
        private void configureButtonBindings() {
                /* Driver Buttons */

                resetGyroButton.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro())); // driver.y().onTrue(new
                                                                                       // InstantCommand(() ->
                                                                                       // s_Swerve.zeroGyro()));

                autoBalanceButton.onTrue(new AutoBalancing(s_Swerve, true));
                followPathButton.whileTrue(
                                autoBuilder.followPath(
                                                segmentLineUp.getTrajectory(SEGMENT.HUMANPLAYER,
                                                                () -> s_Swerve.getPose())));
                selectGamepieceTrigger.onTrue(new SelectCommand(
                                Map.ofEntries(
                                                Map.entry(GamePiece.CUBE,
                                                                new TimedIntake(s_Intake, .5, GamePiece.CUBE,
                                                                                EjectSpeed.NORMAL, Direction.OUTTAKE)),
                                                Map.entry(GamePiece.CONE,
                                                                new SequentialCommandGroup(
                                                                                new InstantCommand(() -> s_Wrist
                                                                                                .setPIDFFMode(PIDFFmode.UNWEIGHTED)),
                                                                                new TimedIntake(s_Intake, .5,
                                                                                                GamePiece.CONE,
                                                                                                EjectSpeed.NORMAL,
                                                                                                Direction.OUTTAKE)))),
                                () -> gamePiece));

                /* Operator Buttons */
                pickStandingConeButton
                                .onTrue(
                                                new SequentialCommandGroup(
                                                                new InstantCommand(() -> setGamePiece(GamePiece.CONE)),
                                                                new SetPosition(s_Wrist, s_Elevator,
                                                                                Position.STANDINGCONEINTAKE,
                                                                                () -> GamePiece.CONE)));

                pickCubeButton
                                .onTrue(new SequentialCommandGroup(
                                                new InstantCommand(() -> setGamePiece(GamePiece.CUBE)),
                                                new SetPosition(s_Wrist, s_Elevator, Position.CUBEINTAKE,
                                                                () -> GamePiece.CUBE)));

                pickTippedConeButton.onTrue(new SequentialCommandGroup(
                                new InstantCommand(() -> setGamePiece(GamePiece.CONE)),
                                new SetPosition(s_Wrist, s_Elevator, Position.TIPPEDCONEINTAKE, () -> GamePiece.CONE)));

               pickHumanPlayerButton
                                .onTrue(new SequentialCommandGroup(
                                                new InstantCommand(() -> setGamePiece(GamePiece.CONE)),
                                                new SetPosition(s_Wrist, s_Elevator, Position.HUMANPLAYERINTAKE,
                                                                () -> GamePiece.CONE)));

                stowIntakeButton
                                .onTrue(new SetPosition(s_Wrist, s_Elevator, Position.STANDBY, () -> getGamePiece()));

                outTakeSlowButton.onTrue(new SelectCommand(
                                Map.ofEntries(
                                                Map.entry(GamePiece.CUBE,
                                                                new TimedIntake(s_Intake, .5, GamePiece.CUBE,
                                                                                EjectSpeed.NORMAL, Direction.OUTTAKE)),
                                                Map.entry(GamePiece.CONE,
                                                                new SequentialCommandGroup(
                                                                                new InstantCommand(() -> s_Wrist
                                                                                                .setPIDFFMode(PIDFFmode.UNWEIGHTED)),
                                                                                new TimedIntake(s_Intake, .5,
                                                                                                GamePiece.CONE,
                                                                                                EjectSpeed.NORMAL,
                                                                                                Direction.OUTTAKE)))),
                                () -> gamePiece));

                outTakeFastButton.onTrue(new SelectCommand(
                                Map.ofEntries(
                                                Map.entry(GamePiece.CUBE,
                                                                new TimedIntake(s_Intake, .5, GamePiece.CUBE,
                                                                                EjectSpeed.FAST, Direction.OUTTAKE)),
                                                Map.entry(GamePiece.CONE,
                                                                new TimedIntake(s_Intake, .5, GamePiece.CONE,
                                                                                EjectSpeed.FAST, Direction.OUTTAKE))),
                                () -> gamePiece));

                toggleLEDButton.onTrue(new InstantCommand(() -> s_LEDs.toggleHPSignal()));

                placeHighButton.onTrue(new SelectCommand(
                                Map.ofEntries(
                                                Map.entry(GamePiece.CUBE,
                                                                new SetPosition(s_Wrist, s_Elevator, Position.HIGH,
                                                                                () -> getGamePiece())),
                                                Map.entry(GamePiece.CONE, new SequentialCommandGroup(
                                                                s_Wrist.setPositionCMD(Position.STANDBY.getWrist()),
                                                                s_Elevator.setPositionCMD(Position.CONEHIGH.getElev()),
                                                                s_Wrist.setPositionCMD(Position.CONEHIGH.getWrist())))),
                                () -> gamePiece));

                placeMidButton.onTrue(new SetPosition(s_Wrist, s_Elevator, Position.MID, () -> getGamePiece()));
                placeLowButton.onTrue(new SetPosition(s_Wrist, s_Elevator, Position.LOW, () -> getGamePiece()));

        }

        private void configureSmartDashboard() {
                // Autonomous Mode Chooser
                autoChooser.setDefaultOption("Move forward", moveForward);
                autoChooser.addOption("S curve", sCurve);
                autoChooser.addOption("Auto balance", autoBalance);
                autoChooser.addOption("Cone and Cube", coneCubeDeposit);
                autoChooser.addOption("Back and Forth", backnForth);
                SmartDashboard.putData(autoChooser);
        }

        /**
         * Ran once the robot is put in disabled
         */
        public void disabledInit() {
                s_Swerve.resetToAbsolute();
                s_LEDs.setLEDMode(LEDMode.VECTORWAVE);
        }

        public static GamePiece getGamePiece() {
                return gamePiece;
        }

        public static void setGamePiece(GamePiece piece) {
                gamePiece = piece;
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */

        public Command getAutonomousCommand() {
                // Executes the autonomous command chosen in smart dashboard
                return new ParallelCommandGroup(
                                new InstantCommand(
                                                () -> s_Swerve.getField().getObject("Field").setTrajectory(
                                                                autoChooser.getSelected())),
                                autoBuilder.fullAuto(autoChooser.getSelected()));
        }
}
