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
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
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
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton IntakePos = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton ArmZero = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton PerimeterShot = new JoystickButton(operator, 10);
    private final JoystickButton shootButton = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton spit = new JoystickButton(operator, 7);
    private final JoystickButton forceIntake = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton amp = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton limeDrive = new JoystickButton(driver, 9);
    public boolean gyroCheck;
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        new InstantCommand((() -> gyroCheck = s_Swerve.getHeading().getDegrees() >= -90 && s_Swerve.getHeading().getDegrees() <= 90));
        SmartDashboard.putBoolean("RobotGyro", gyroCheck);
        NamedCommands.registerCommand("IntakePos", new TeleopArm(arm, intakePivot, -45, 99).withTimeout(2));
        NamedCommands.registerCommand("Intake", new TeleopIntake(intake, 1, 2, gyroCheck));
        NamedCommands.registerCommand("Shoot", new TeleopIntake(intake, -0.1, 0.2, gyroCheck).andThen(new TeleopShooter(shooter, 0.45, 1)).andThen(new ParallelCommandGroup(new TeleopIntake(intake, 1, 1, gyroCheck), new TeleopShooter(shooter, 0.6, 1))));
        NamedCommands.registerCommand("ShootSecond", new TeleopIntake(intake, -0.1, 0.2, gyroCheck).andThen(new TeleopShooter(shooter, 0.4, 1)).andThen(new ParallelCommandGroup(new TeleopIntake(intake, 1, 1, gyroCheck), new TeleopShooter(shooter, 0.6, 1))));
        NamedCommands.registerCommand("ArmZero", new TeleopArm(arm, intakePivot, 0, 0).withTimeout(2));
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> -driver.getRawAxis(rotationAxis),
                () -> robotCentric.getAsBoolean()
            )
        );
        climber.setDefaultCommand(
            new TeleopClimb(climber,
             () -> -operator.getRawAxis(climbAxis))
        );

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        // Configure the button bindings
        configureButtonBindings();

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        // IntakePos.onTrue(Commands.runOnce(()->{
        //     // Degrees
        //     Arm.setAngle(90);
        // },
        // Arm));
        // ArmZero.onTrue(Commands.runOnce(()->{
        //     Arm.setAngle(0);

        // },
        // Arm));
        
        shootButton.onTrue(new TeleopIntake(intake, -0.1, 0.2, gyroCheck).andThen(new TeleopShooter(shooter, 0.45, 1)).andThen(new ParallelCommandGroup(new TeleopIntake(intake, 1, 1, gyroCheck), new TeleopShooter(shooter, 0.6, 1))));
        PerimeterShot.onTrue(new ParallelCommandGroup(new TeleopArm(arm, intakePivot, -35, 0), (new TeleopIntake(intake, -0.1, 0.2, gyroCheck).andThen(new TeleopShooter(shooter, 0.8, 1)).andThen(new ParallelCommandGroup(new TeleopIntake(intake, 1, 1, gyroCheck), new TeleopShooter(shooter, 9, 1))))));
        spit.whileTrue(new TeleopIntake(intake, -1, 0, gyroCheck));
        forceIntake.whileTrue(new TeleopIntake(intake, 1, 0, gyroCheck));
        // intake90.onTrue(new InstantCommand(()-> intakePivot.setAngle(90, 0)));
        // intakeZero.onTrue(new InstantCommand(()-> intakePivot.setAngle(0, 0)));
        IntakePos.onTrue(new TeleopArm(arm, intakePivot, -45, 99));
        ArmZero.onTrue(new TeleopArm(arm, intakePivot, 0, 0));
        amp.onTrue(new ParallelCommandGroup(new TeleopIntake(intake, -0.1, 0.25, gyroCheck), new TeleopArm(arm, intakePivot, -25, 15).withTimeout(1)).andThen(new TeleopIntake(intake, -0.3, 1, gyroCheck)));
        limeDrive.whileTrue(new TeleopLimelightDrive(s_Swerve, limelight, intakePivot, false));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
