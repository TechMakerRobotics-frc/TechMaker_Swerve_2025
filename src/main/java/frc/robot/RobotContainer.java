package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.drive.*;
import frc.robot.commands.flywheel.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.leds.*;
import frc.robot.commands.lockwheel.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.flywheel.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.lockwheel.*;
import frc.robot.subsystems.vision.*;
import frc.robot.util.RegisNamedCommands;
import frc.robot.util.RobotModeTo;
import frc.robot.util.StateMachine;
import frc.robot.util.zones.ZoneManager;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    public final Vision vision;
    private final Flywheel flywheel;
    private final Intake intake;
    private final Lockwheel lockwheel;
    public final Leds leds;

    // control leds
    private int currentLedState = 0;
    private final Command[] ledCommands;

    public final ZoneManager zones;
    public final StateMachine stateMachine;

    private SwerveDriveSimulation driveSimulation = null;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Operator Controller
    private final CommandXboxController OperatorController = new CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;
    private final LoggedDashboardChooser<Command> robotModeChooser;

    // tunable flywheel velocity
    private LoggedNetworkNumber flywheelSpeedInside = new LoggedNetworkNumber("/Tuning/Flywheel Speed Inside", 300.0);
    private LoggedNetworkNumber flywheelSpeedOutside =
            new LoggedNetworkNumber("/Tuning/Flywheel Speed Outside", 3000.0);

    // tunable intake velocity
    private LoggedNetworkNumber intakeSpeedInside = new LoggedNetworkNumber("/Tuning/Intake Speed Inside", 600);
    private LoggedNetworkNumber intakeSpeedOutside = new LoggedNetworkNumber("/Tuning/Intake Speed Outside", 600);

    // tunable lockwheel velocity
    private LoggedNetworkNumber lockwheelSpeedInside = new LoggedNetworkNumber("/Tuning/Flywheel Speed Inside", 800);
    private LoggedNetworkNumber lockwheelSpeedOutside = new LoggedNetworkNumber("/Tuning/Flywheel Speed Outside", 800);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOSparkAndTalonFXReal(TunerConstants.FrontLeft, 0),
                        new ModuleIOSparkAndTalonFXReal(TunerConstants.FrontRight, 1),
                        new ModuleIOSparkAndTalonFXReal(TunerConstants.BackLeft, 2),
                        new ModuleIOSparkAndTalonFXReal(TunerConstants.BackRight, 3));
                this.vision = new Vision(drive, new VisionIOPhotonVision(FL_CAM_NAME, ROBOT_TO_FL_CAM));
                new VisionIOPhotonVision(FR_CAM_NAME, ROBOT_TO_FR_CAM);
                new VisionIOPhotonVision(LIMELIGHT_NAME, ROBOT_TO_FR_CAM);
                flywheel = new Flywheel(new FlywheelIOVictorSPX());
                intake = new Intake(new IntakeIOSparkMax());
                lockwheel = new Lockwheel(new LockwheelIOVictorSPX());
                leds = new Leds();
                new RegisNamedCommands(flywheel, intake, lockwheel, drive, leds);
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations

                driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackRight, driveSimulation.getModules()[3]));
                vision = new Vision(
                        drive,
                        new VisionIOPhotonVisionSim(
                                FL_CAM_NAME, ROBOT_TO_FL_CAM, driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                                FR_CAM_NAME, ROBOT_TO_FR_CAM, driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                                LIMELIGHT_NAME, ROBOT_TO_LIMELIGHT, driveSimulation::getSimulatedDriveTrainPose));
                flywheel = new Flywheel(new FlywheelIOSim());
                intake = new Intake(new IntakeIOSim(driveSimulation));
                lockwheel = new Lockwheel(new LockwheelIOSim());

                leds = new Leds();
                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {}, new VisionIO() {});
                flywheel = new Flywheel(new FlywheelIO() {});
                intake = new Intake(new IntakeIO() {});
                lockwheel = new Lockwheel(new LockwheelIO() {});
                leds = new Leds();
                break;
        }

        zones = new ZoneManager(drive);
        stateMachine = new StateMachine(zones, drive, lockwheel, flywheel);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Align to 5 X, 5 Y", new AlignTo(drive, 5, 5, 5));
        autoChooser.addOption("Align to Tag 7", new AlignTo(drive, 7, 5));
        autoChooser.addOption("Drive to 5 X, 5 Y", new DriveTo(5, 5, 90, 15));
        autoChooser.addOption("AutoChoreo", new ChoreoAuto("FirstAuto", 20));

        robotModeChooser = new LoggedDashboardChooser<>("Robot Mode", AutoBuilder.buildAutoChooser());

        robotModeChooser.addDefaultOption("Robot Mode Brake", new RobotModeTo("Brake", drive));
        robotModeChooser.addOption("Robot Mode Coast", new RobotModeTo("Coast", drive));

        ledCommands = new Command[] {
            new LedRed(leds),
            new LedGreen(leds),
            new LedBlue(leds),
            new LedOff(leds),
            new LedWhite(leds),
            new LedYellow(leds),
            new LedCian(leds),
            new LedRainbow(leds)
        };

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
                ? () -> drive.setPose(
                        driveSimulation
                                .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
                : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro

        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()));

        // Lock to 0° when A button is held
        controller
                .a()
                .whileTrue(DriveCommands.joystickDriveAtAngle(
                        drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> new Rotation2d()));

        new Trigger(() -> stateMachine.isReadyToAlign())
                .whileTrue(DriveCommands.joystickDriveAtPoint(
                        drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX(), 0, 5.5));

        // Switch to X pattern when X button is pressed
        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro / odometry
        controller.povRight().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        // Operator commands

        // flywheel
        OperatorController.y()
                .onTrue(new OutsideFlywheelCommand(flywheel, flywheelSpeedOutside.get()))
                .onFalse(new StopFlywheelCommand(flywheel));
        OperatorController.a()
                .onTrue(new InsideFlywheelCommand(flywheel, flywheelSpeedInside.get()))
                .onFalse(new StopFlywheelCommand(flywheel));

        // intake
        OperatorController.povUp()
                .onTrue(new InsideIntakeCommand(intake, intakeSpeedInside.get()))
                .onFalse(new StopIntakeCommand(intake));
        OperatorController.povDown()
                .onTrue(new OutsideIntakeCommand(intake, intakeSpeedOutside.get()))
                .onFalse(new StopIntakeCommand(intake));

        OperatorController.povLeft().onFalse(new ExtendIntakeCommand(intake));
        OperatorController.povRight().onTrue(new RetractIntakeCommand(intake));

        OperatorController.leftBumper().onTrue(new AlignBall(lockwheel));

        // lockwheel
        OperatorController.x()
                .onTrue(new OutsideLockwheelCommand(lockwheel, lockwheelSpeedInside.get()))
                .onFalse(new StopLockwheelCommand(lockwheel));
        OperatorController.b()
                .onTrue(new InsideLockwheelCommand(lockwheel, lockwheelSpeedOutside.get()))
                .onFalse(new StopLockwheelCommand(lockwheel));

        // leds
        OperatorController.leftStick().onTrue(Commands.runOnce(() -> {
            // Alterna o comando atual para o próximo na lista
            currentLedState = (currentLedState + 1) % ledCommands.length;
            Command nextCommand = ledCommands[currentLedState];

            // Executa o próximo comando
            nextCommand.schedule();
        }));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void displaySimFieldToAdvantageScope() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Notes", SimulatedArena.getInstance().getGamePiecesArrayByType("Note"));
    }
}
