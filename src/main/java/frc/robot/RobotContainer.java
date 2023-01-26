package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.AutoAlign;
import frc.robot.subsystems.drive.commands.Drive;
import frc.robot.subsystems.drive.commands.ResetGyro;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indexer.commands.Index;
import frc.robot.subsystems.indexer.commands.IndexFast;
import frc.robot.subsystems.indexer.commands.IndexShooter;
import frc.robot.subsystems.indexer.commands.Outdex;
import frc.robot.subsystems.leds.LedSubsystem;
import frc.robot.subsystems.leds.commands.SabotageText;
import frc.robot.subsystems.leds.commands.SetLeds;

public class RobotContainer {
    // The robot's subsystems
    public static final DriveSubsystem swerve = new DriveSubsystem();
    public static final IndexerSubsystem indexer = new IndexerSubsystem();
    public static final LedSubsystem leds = new LedSubsystem();

    // The driver's controller
    XboxController driverController = new XboxController(0);
    XboxController operatorController = new XboxController(1);
    Joystick joystick = new Joystick(0);
    Joystick opJoystick = new Joystick(1);

    private boolean fieldRelative = true;

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    private double xSpeed;
    private double ySpeed;
    private double rot;

    public static final double x = 0.257175; // 10.125"
    public static final double y = 0.32146875; // 12.65625"

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(new Translation2d(y, x),
            new Translation2d(y, -x), new Translation2d(-y, x), new Translation2d(-y, -x));

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        swerve.initDefaultCommand();
        leds.initDefaultCommand();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * calling passing it to a {@link JoystickButton}.
     */
    public void configureButtonBindings() {

        // Driver Buttons
        JoystickButton startButton = new JoystickButton(joystick, 8);
        startButton.whenPressed(new ResetGyro(swerve));
        JoystickButton selectButton = new JoystickButton(joystick, 7);
        selectButton.whenPressed(() -> {
            System.out.println("Toggled field centric!");
            fieldRelative = !fieldRelative;
        });
        JoystickButton leftBumper = new JoystickButton(joystick, 5);
        JoystickButton yButton = new JoystickButton(joystick, 4);

        yButton.whenActive(new AutoAlign(swerve));
        yButton.whenInactive(new Drive(swerve, fieldRelative));

        JoystickButton oLeftBumper = new JoystickButton(opJoystick, 5);

        Command shoot = new IndexShooter(indexer);

        Button down = new Button() {
            public boolean get() {
                return driverController.getPOV() == 180;
            }
        };

        Button rightTrigger = new Button() {

            public boolean get() {
                return driverController.getRightTriggerAxis() > 0.1;
            }

        };

        // rightTrigger.whenHeld(new Intake(intake, indexer));

        Button indexerSensorTriggered = new Button() {
            public boolean get() {
                if (!rightTrigger.get() && (indexer.getProx1() == false && indexer.getProx2() == false)) {
                    return false;
                } else if (leftBumper.get() || oLeftBumper.get()) {
                    return false;
                } else if (rightTrigger.get() || indexer.getProx1() == false) {
                    return true;
                } else {
                    return false;
                }
            }
        };

        Button leftTrigger = new Button() {

            public boolean get() {
                return driverController.getLeftTriggerAxis() > 0.1;
            }

        };

        // Operator Buttons
        // SpinnerStage2 stage2 = new SpinnerStage2(spinner);
        // SpinnerStage2 stage3 = new SpinnerStage3(spinner);
        // ParallelCommandGroup spinClose = new ParallelCommandGroup(new

        JoystickButton oAButton = new JoystickButton(opJoystick, 1);
        JoystickButton oBButton = new JoystickButton(opJoystick, 2);
        JoystickButton oRBButton = new JoystickButton(opJoystick, 6);
        JoystickButton oXButton = new JoystickButton(opJoystick, 3);
        JoystickButton oYButton = new JoystickButton(opJoystick, 4);

        Button raiseClimber = new Button(() -> oRBButton.get() && oXButton.get());

        Button lowerClimber = new Button(() -> oRBButton.get() && oYButton.get());

        Button leftY = new Button() {

            public boolean get() {
                return Math.abs(operatorController.getLeftY()) > 0.15;
            }

        };

        // TODO add spinner cancel option
        // Button oRight = new Button() {
        // public boolean get() {
        // return driverController.getPOV() == 90;
        // }
        // };

        // BooleanSupplier spinnerAction;
        // if(stage2.isScheduled()) {
        // spinnerAction = () -> true;
        // } else {
        // spinnerAction = () -> false;
        // }

        // ConditionalCommand spinnerCancelCommand = new
        // ConditionalCommand(stage2.cancel(), null, spinnerAction);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        CommandScheduler.getInstance().clearButtons();

        swerve.updateOdometry();
        return null;

        // Run path following command, then stop at the end.
    }
}
