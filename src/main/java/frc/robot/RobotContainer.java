package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.exampleAuto;
import frc.robot.commands.TeleopArm;
import frc.robot.commands.TeleopIntake;
import frc.robot.commands.TeleopShooter;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
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
   // public final Climber climber = new Climber();
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
    private final JoystickButton ArmPosition1 = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton ArmPosition2 = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton intakeButton = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton shootButton = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton sameShoot = new JoystickButton(operator, XboxController.Button.kBack.value);
    private final JoystickButton intakeZero = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton intake90 = new JoystickButton(operator, XboxController.Button.kRightBumper.value);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
        // climber.setDefaultCommand(
        //     new TeleopClimb(climber,
        //      () -> operator.getRawAxis(climbAxis))
        // );
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
        // ArmPosition1.onTrue(Commands.runOnce(()->{
        //     // Degrees
        //     Arm.setAngle(90);
        // },
        // Arm));
        // ArmPosition2.onTrue(Commands.runOnce(()->{
        //     Arm.setAngle(0);

        // },
        // Arm));
        intakeButton.onTrue(new TeleopIntake(intake, 0.5, 0.3));
        shootButton.onTrue(new TeleopIntake(intake, -0.1, 0.2).andThen(new TeleopShooter(shooter, 1, 2)).andThen(new ParallelCommandGroup(new TeleopIntake(intake, 1, 1), new TeleopShooter(shooter, 1, 1))));
        sameShoot.onTrue(new ParallelCommandGroup(new TeleopShooter(shooter, 1, 3), new TeleopIntake(intake, 0.5, 3)));
        // intake90.onTrue(new InstantCommand(()-> intakePivot.setAngle(90, 0)));
        // intakeZero.onTrue(new InstantCommand(()-> intakePivot.setAngle(0, 0)));
        ArmPosition1.onTrue(new TeleopArm(arm, intakePivot, -60, 100));
        ArmPosition2.onTrue(new TeleopArm(arm, intakePivot, 0, 0));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
