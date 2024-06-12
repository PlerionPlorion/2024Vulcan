package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.AmpArm;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoShooter;
import frc.robot.commands.ManualIntake;
import frc.robot.commands.TeleopArm;
import frc.robot.commands.TeleopClimb;
import frc.robot.commands.TeleopIntake;
import frc.robot.commands.TeleopLimelightDrive;
import frc.robot.commands.TeleopShooter;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        /* Subsystems */
        private final Swerve s_Swerve = new Swerve();
        private final Arm arm = new Arm();
        public final Shooter shooter = new Shooter();
        public final Intake intake = new Intake();
        public final IntakePivot intakePivot = new IntakePivot();
        public final Climber climber = new Climber();
        public final Limelight limelight = new Limelight();
        /* PathPlanner */
        private final SendableChooser<Command> autoChooser;

        /* Controllers */
        private final Joystick driver = new Joystick(0);
        private final Joystick operator = new Joystick(1);

        /* Drive Controls */
        private final int translationAxis = XboxController.Axis.kLeftY.value;
        private final int strafeAxis = XboxController.Axis.kLeftX.value;
        private final int rotationAxis = XboxController.Axis.kRightX.value;
        private final int climbAxis = XboxController.Axis.kRightY.value;
        /* Driver Buttons */
        private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
        private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kStart.value);
        // private final JoystickButton IntakePos = new JoystickButton(operator, XboxController.Button.kX.value);
        private final JoystickButton ArmZero = new JoystickButton(operator, XboxController.Button.kB.value);
        private final JoystickButton PerimeterShot = new JoystickButton(operator, 10);
        private final JoystickButton shootButton = new JoystickButton(operator, XboxController.Button.kA.value);
        private final JoystickButton spit = new JoystickButton(operator, 7);
        private final JoystickButton forceIntake = new JoystickButton(operator, XboxController.Button.kY.value);
        private final JoystickButton amp = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
        private final JoystickButton limeDrive = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

        // public boolean gyroCheck;
        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                SmartDashboard.putBoolean("RobotGyro", s_Swerve.getGyroCheck());

                NamedCommands.registerCommand("IntakePos", new TeleopArm(arm, intakePivot, -60, 98).withTimeout(1));

                NamedCommands.registerCommand("ShootPos", new TeleopArm(arm, intakePivot, -65, 12).withTimeout(1));

                NamedCommands.registerCommand("FastShootPos", new TeleopArm(arm, intakePivot, -65, 12).withTimeout(0.5));

                NamedCommands.registerCommand("Intake", new ManualIntake(intake, 1, 0.7));

                NamedCommands.registerCommand("Shoot",
                                new AutoIntake(intake, -0.1, 0.2)
                        .andThen(new AutoShooter(intake, 1, 0.25))
                        .andThen(new ParallelCommandGroup(
                        new AutoIntake(intake, 1, 1),
                        new AutoShooter(intake, 1, 1))));

                // NamedCommands.registerCommand("ShootSecond",
                //                 new TeleopIntake(intake, -0.1, 0.2, s_Swerve::getGyroCheck)
                //                                 .andThen(new TeleopShooter(shooter, 0.4, 1))
                //                                 .andThen(new ParallelCommandGroup(
                //                                                 new TeleopIntake(intake, 1, 1, s_Swerve::getGyroCheck),
                //                                                 new TeleopShooter(shooter, 0.6, 1))));

                NamedCommands.registerCommand("ArmZero", new TeleopArm(arm, intakePivot, 0, 0).withTimeout(2));

                s_Swerve.setDefaultCommand(
                                new TeleopSwerve(
                                                s_Swerve,
                                                () -> -driver.getRawAxis(translationAxis),
                                                () -> -driver.getRawAxis(strafeAxis),
                                                () -> -driver.getRawAxis(rotationAxis),
                                                () -> robotCentric.getAsBoolean()));

                climber.setDefaultCommand(
                                new TeleopClimb(climber,
                                                () -> -operator.getRawAxis(climbAxis)));

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
                // Configure the button bindings
                configureButtonBindings();

        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */

        private void configureButtonBindings() {
                /* Driver Buttons */
                zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
                // IntakePos.onTrue(Commands.runOnce(()->{
                // // Degrees
                // Arm.setAngle(90);
                // },
                // Arm));
                // ArmZero.onTrue(Commands.runOnce(()->{
                // Arm.setAngle(0);

                // },
                // Arm));

                shootButton.onTrue(new TeleopIntake(intake, -0.1, 0.2, s_Swerve::getGyroCheck)
                                .andThen(new TeleopShooter(intake, 1, 1, s_Swerve::getGyroCheck))
                                .andThen(new ParallelCommandGroup(
                                                new TeleopIntake(intake, 1, 1, s_Swerve::getGyroCheck),
                                                new TeleopShooter(intake, 1, 1, s_Swerve::getGyroCheck))));

                shootButton.onTrue(new TeleopIntake(intake, -0.15, 0.24, s_Swerve::getGyroCheck)
                .andThen(new TeleopShooter(intake, 1, 0.25, s_Swerve::getGyroCheck))
                .andThen(new ParallelCommandGroup(
                                new TeleopIntake(intake, 1, 1, s_Swerve::getGyroCheck),
                                new TeleopShooter(intake, 1, 1, s_Swerve::getGyroCheck))));

                // PerimeterShot.onTrue(new ParallelCommandGroup(new TeleopArm(arm, intakePivot, -35, 0),
                //                 (new TeleopIntake(intake, -0.1, 0.2, s_Swerve::getGyroCheck)
                //                                 .andThen(new TeleopShooter(shooter, 0.8, 1))
                //                                 .andThen(new ParallelCommandGroup(
                //                                                 new TeleopIntake(intake, 1, 1, s_Swerve::getGyroCheck),
                //                                                 new TeleopShooter(shooter, 9, 1))))));
                new POVButton(operator, 0)
                        .onTrue(new TeleopArm(arm, intakePivot, -60, 98));
                new POVButton(operator, 180)
                        .onTrue(new TeleopArm(arm, intakePivot, 60, -98));
                spit.whileTrue(new ManualIntake(intake, -1, 0));

                forceIntake.whileTrue(new ManualIntake(intake, 1, 0));
                // intake90.onTrue(new InstantCommand(()-> intakePivot.setAngle(90, 0)));
                // intakeZero.onTrue(new InstantCommand(()-> intakePivot.setAngle(0, 0)));
                // IntakePos.onTrue(new TeleopArm(arm, intakePivot, -45, 99));
                ArmZero.onTrue(new TeleopArm(arm, intakePivot, 0, 0));

                amp.onTrue(new AmpArm(arm, intakePivot, climber, -40, 20, 100, s_Swerve::getAmpGyroCheck).withTimeout(1));
                amp.onFalse(new TeleopIntake(intake, -0.3, 1, s_Swerve::getAmpGyroCheck)
                .andThen(new AmpArm(arm, intakePivot, climber, 0, 0, 0, s_Swerve::getAmpGyroCheck).withTimeout(1)));

                limeDrive.whileTrue(new TeleopLimelightDrive(s_Swerve, limelight, intakePivot, arm, false));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // return autoChooser.getSelected();
                return null;
        }
}
