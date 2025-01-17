package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
    private static final double DEADBAND = 0.1;
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    private DriveCommands() {}

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }

    /** Field relative drive command using two joysticks (controlling linear and angular velocities). */
    public static Command joystickDrive(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
        return Commands.run(
                () -> {
                    // Get linear velocity
                    Translation2d linearVelocity =
                            getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                    // Apply rotation deadband
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    // Square rotation value for more precise control
                    omega = Math.copySign(omega * omega, omega);

                    // Convert to field relative speeds & send command
                    ChassisSpeeds speeds = new ChassisSpeeds(
                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                            omega * drive.getMaxAngularSpeedRadPerSec());
                    boolean isFlipped = DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red;
                    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, 
                            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation());
                    drive.runVelocity(speeds);
                },
                drive);
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control. Possible use cases
     * include snapping to an angle, aiming at a vision target, or controlling absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> rotationSupplier) {

        // Create PID controller
        ProfiledPIDController angleController = new ProfiledPIDController(
                ANGLE_KP, 0.0, ANGLE_KD, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.run(
                        () -> {
                            // Get linear velocity
                            Translation2d linearVelocity =
                                    getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                            // Calculate angular speed
                            double omega = angleController.calculate(
                                    drive.getRotation().getRadians(),
                                    rotationSupplier.get().getRadians());

                            // Convert to field relative speeds & send command
                            ChassisSpeeds speeds = new ChassisSpeeds(
                                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                    omega);
                            boolean isFlipped = DriverStation.getAlliance().isPresent()
                                    && DriverStation.getAlliance().get() == Alliance.Red;
                            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation());
                            drive.runVelocity(speeds);
                        },
                        drive)

                // Reset PID controller when command starts
                .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
    }

    public static Command joystickDriveAimAtPoint(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, double targetX, double targetY) {

        // Criação do controlador PID para controle de rotação
        ProfiledPIDController angleController = new ProfiledPIDController(
                ANGLE_KP, 0.0, ANGLE_KD, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construção do comando
        return Commands.run(
                        () -> {
                            // Obter velocidade linear a partir dos joysticks
                            Translation2d linearVelocity =
                                    getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                            // Pose atual do robô
                            Pose2d currentPose = drive.getPose();

                            // Calcula o ângulo desejado para o ponto (x, y)
                            double desiredTheta = Math.PI + 
                                    (Math.atan2(targetY - currentPose.getY(), targetX - currentPose.getX()));

                            // Apply rotation deadband
                            double omegaController = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                            // Square rotation value for more precise control
                            omegaController = Math.copySign(omegaController * omegaController, omegaController);

                            // Calcula a velocidade angular usando o controlador PID
                            double omega = angleController.calculate(
                                    drive.getRotation().getRadians(), desiredTheta);

                            if (!(omegaController == 0)) {
                                omega = omegaController * drive.getMaxAngularSpeedRadPerSec();
                            }

                            // Converte as velocidades para referencia de campo e envia o comando
                            ChassisSpeeds speeds = new ChassisSpeeds(
                                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                    omega);
                            boolean isFlipped = DriverStation.getAlliance().isPresent()
                                    && DriverStation.getAlliance().get() == Alliance.Red;
                            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation());
                            drive.runVelocity(speeds);
                        },
                        drive)

                // Reseta o controlador PID quando o comando inicia
                .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
    }
    public static Command joystickDriveTowardsPoint(
        Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, double targetX, double targetY) {

    final double targetWeight = 0.3; // 30% para o alvo, 70% para os joysticks

    return Commands.run(
            () -> {
                double joystickX = xSupplier.getAsDouble();
                double joystickY = ySupplier.getAsDouble();

                // Apply rotation deadband
                double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                // Square rotation value for more precise control
                omega = Math.copySign(omega * omega, omega);

                // Aplicar deadband
                joystickX = MathUtil.applyDeadband(joystickX, DEADBAND);
                joystickY = MathUtil.applyDeadband(joystickY, DEADBAND);

                // Posição atual do robô
                Pose2d currentPose = drive.getPose();
                double currentX = currentPose.getX();
                double currentY = currentPose.getY();

                // Calcular vetor para o ponto alvo
                double targetVectorX = targetX - currentX;
                double targetVectorY = targetY - currentY;

                // Normalizar o vetor do alvo
                double targetMagnitude = Math.hypot(targetVectorX, targetVectorY);
                if (targetMagnitude > 0.01) {
                    targetVectorX /= targetMagnitude;
                    targetVectorY /= targetMagnitude;
                }

                // Combinar vetores (joystick e direção ao alvo)
                double blendedX = (1 - targetWeight) * joystickX + targetWeight * targetVectorX;
                double blendedY = (1 - targetWeight) * joystickY + targetWeight * targetVectorY;

                // Normalizar o vetor combinado (opcional, se necessário)
                double blendedMagnitude = Math.hypot(blendedX, blendedY);
                if (blendedMagnitude > 1.0) {
                    blendedX /= blendedMagnitude;
                    blendedY /= blendedMagnitude;
                }

                // Ajuste para aliança (inversão para aliança vermelha)
                boolean isFlipped = DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red;

                // Se os joysticks estão parados, não realiza movimento
                if (joystickX == 0.0 && joystickY == 0.0) {
                        blendedX = 0;
                        blendedY = 0;
                }
                // Transformar velocidades para referência de campo
                ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        blendedX * drive.getMaxLinearSpeedMetersPerSec(),
                        blendedY * drive.getMaxLinearSpeedMetersPerSec(),
                        omega * drive.getMaxAngularSpeedRadPerSec(), // Sem rotação
                        isFlipped
                                ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                : drive.getRotation());
                drive.runVelocity(speeds);
            },
            drive);
}




    public static Command joystickDriveAimAtPoint(
        Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, Pose2d targetPose) {

        // Criação do controlador PID para controle de rotação
        ProfiledPIDController angleController = new ProfiledPIDController(
                ANGLE_KP, 0.0, ANGLE_KD, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construção do comando
        return Commands.run(
                        () -> {
                        // Obter velocidade linear a partir dos joysticks
                        Translation2d linearVelocity =
                                getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                        // Pose atual do robô
                        Pose2d currentPose = drive.getPose();

                        // Calcula o ângulo desejado para o ponto (x, y)
                        double desiredTheta = Math.PI + 
                                (Math.atan2(targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX()));

                        // Apply rotation deadband
                        double omegaController = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                        // Square rotation value for more precise control
                        omegaController = Math.copySign(omegaController * omegaController, omegaController);

                        // Calcula a velocidade angular usando o controlador PID
                        double omega = angleController.calculate(
                                drive.getRotation().getRadians(), desiredTheta);

                        if (!(omegaController == 0)) {
                            omega = omegaController * drive.getMaxAngularSpeedRadPerSec();
                        }

                        // Converte as velocidades para referencia de campo e envia o comando
                        ChassisSpeeds speeds = new ChassisSpeeds(
                                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                omega);
                        boolean isFlipped = DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get() == Alliance.Red;
                        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
                                isFlipped
                                        ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                        : drive.getRotation());
                        drive.runVelocity(speeds);
                    },
                    drive)

            // Reseta o controlador PID quando o comando inicia
            .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
        }

    public static Command joystickDriveTowardsPoint(
        Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, double targetX, double targetY) {

    final double targetWeight = 0.3; // 30% para o alvo, 70% para os joysticks

    return Commands.run(
            () -> {
                double joystickX = xSupplier.getAsDouble();
                double joystickY = ySupplier.getAsDouble();

                // Apply rotation deadband
                double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                // Square rotation value for more precise control
                omega = Math.copySign(omega * omega, omega);

                // Aplicar deadband
                joystickX = MathUtil.applyDeadband(joystickX, DEADBAND);
                joystickY = MathUtil.applyDeadband(joystickY, DEADBAND);

                // Posição atual do robô
                Pose2d currentPose = drive.getPose();
                double currentX = currentPose.getX();
                double currentY = currentPose.getY();

                // Calcular vetor para o ponto alvo
                double targetVectorX = targetX - currentX;
                double targetVectorY = targetY - currentY;

                // Normalizar o vetor do alvo
                double targetMagnitude = Math.hypot(targetVectorX, targetVectorY);
                if (targetMagnitude > 0.01) {
                    targetVectorX /= targetMagnitude;
                    targetVectorY /= targetMagnitude;
                }

                // Combinar vetores (joystick e direção ao alvo)
                double blendedX = (1 - targetWeight) * joystickX + targetWeight * targetVectorX;
                double blendedY = (1 - targetWeight) * joystickY + targetWeight * targetVectorY;

                // Normalizar o vetor combinado (opcional, se necessário)
                double blendedMagnitude = Math.hypot(blendedX, blendedY);
                if (blendedMagnitude > 1.0) {
                    blendedX /= blendedMagnitude;
                    blendedY /= blendedMagnitude;
                }

                // Ajuste para aliança (inversão para aliança vermelha)
                boolean isFlipped = DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red;

                // Se os joysticks estão parados, não realiza movimento
                if (joystickX == 0.0 && joystickY == 0.0) {
                        blendedX = 0;
                        blendedY = 0;
                }
                // Transformar velocidades para referência de campo
                ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        blendedX * drive.getMaxLinearSpeedMetersPerSec(),
                        blendedY * drive.getMaxLinearSpeedMetersPerSec(),
                        omega * drive.getMaxAngularSpeedRadPerSec(), // Sem rotação
                        isFlipped
                                ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                : drive.getRotation());
                drive.runVelocity(speeds);
            },
            drive);
        }

        public static Command joystickDriveTowardsAimAtPoint(
                Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, double targetX, double targetY) {
        
            final double targetWeight = 0.5;

            final ProfiledPIDController angleController = new ProfiledPIDController(
                ANGLE_KP, 0.0, ANGLE_KD, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
            angleController.enableContinuousInput(-Math.PI, Math.PI);
        
            return Commands.run(
                    () -> {
                        double joystickX = -xSupplier.getAsDouble();
                        double joystickY = -ySupplier.getAsDouble();
        
                        joystickX = MathUtil.applyDeadband(joystickX, DEADBAND);
                        joystickY = MathUtil.applyDeadband(joystickY, DEADBAND);
        
                        Pose2d currentPose = drive.getPose();
                        double currentX = currentPose.getX();
                        double currentY = currentPose.getY();
        
                        double targetVectorX = targetX - currentX;
                        double targetVectorY = targetY - currentY;
        
                        double targetMagnitude = Math.hypot(targetVectorX, targetVectorY);
                        if (targetMagnitude > 0.01) {
                            targetVectorX /= targetMagnitude;
                            targetVectorY /= targetMagnitude;
                        }
        
                        double blendedX = (1 - targetWeight) * joystickX + targetWeight * targetVectorX;
                        double blendedY = (1 - targetWeight) * joystickY + targetWeight * targetVectorY;
        
                        double blendedMagnitude = Math.hypot(blendedX, blendedY);
                        if (blendedMagnitude > 1.0) {
                            blendedX /= blendedMagnitude;
                            blendedY /= blendedMagnitude;
                        }
        
                        boolean isFlipped = DriverStation.getAlliance().isPresent()
                                    && DriverStation.getAlliance().get() == Alliance.Blue;
        
                        if (joystickX == 0.0 && joystickY == 0.0) {
                                blendedX = 0;
                                blendedY = 0;
                        }

                        double desiredTheta = Math.PI + 
                                (Math.atan2(targetY - currentPose.getY(), targetX - currentPose.getX()));

                        double omegaController = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                        omegaController = Math.copySign(omegaController * omegaController, omegaController);

                        double omega = angleController.calculate(
                                drive.getRotation().getRadians(), desiredTheta);

                        if (!(omegaController == 0)) {
                            omega = omegaController * drive.getMaxAngularSpeedRadPerSec();
                        }

                        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                blendedX * drive.getMaxLinearSpeedMetersPerSec(),
                                blendedY * drive.getMaxLinearSpeedMetersPerSec(),
                                omega,
                                isFlipped
                                        ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                        : drive.getRotation());
                        drive.runVelocity(speeds);
                    },
                    drive);
            }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data
                Commands.runOnce(() -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),

                // Allow modules to orient
                Commands.run(
                                () -> {
                                    drive.runCharacterization(0.0);
                                },
                                drive)
                        .withTimeout(FF_START_DELAY),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(
                                () -> {
                                    double voltage = timer.get() * FF_RAMP_RATE;
                                    drive.runCharacterization(voltage);
                                    velocitySamples.add(drive.getFFCharacterizationVelocity());
                                    voltageSamples.add(voltage);
                                },
                                drive)

                        // When cancelled, calculate and print results
                        .finallyDo(() -> {
                            int n = velocitySamples.size();
                            double sumX = 0.0;
                            double sumY = 0.0;
                            double sumXY = 0.0;
                            double sumX2 = 0.0;
                            for (int i = 0; i < n; i++) {
                                sumX += velocitySamples.get(i);
                                sumY += voltageSamples.get(i);
                                sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                            }
                            double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                            double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                            NumberFormat formatter = new DecimalFormat("#0.00000");
                            System.out.println("********** Drive FF Characterization Results **********");
                            System.out.println("\tkS: " + formatter.format(kS));
                            System.out.println("\tkV: " + formatter.format(kV));
                        }));
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                // Drive control sequence
                Commands.sequence(
                        // Reset acceleration limiter
                        Commands.runOnce(() -> {
                            limiter.reset(0.0);
                        }),

                        // Turn in place, accelerating up to full speed
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                                },
                                drive)),

                // Measurement sequence
                Commands.sequence(
                        // Wait for modules to fully orient before starting measurement
                        Commands.waitSeconds(1.0),

                        // Record starting measurement
                        Commands.runOnce(() -> {
                            state.positions = drive.getWheelRadiusCharacterizationPositions();
                            state.lastAngle = drive.getRotation();
                            state.gyroDelta = 0.0;
                        }),

                        // Update gyro delta
                        Commands.run(() -> {
                                    var rotation = drive.getRotation();
                                    state.gyroDelta += Math.abs(
                                            rotation.minus(state.lastAngle).getRadians());
                                    state.lastAngle = rotation;
                                })

                                // When cancelled, calculate and print results
                                .finallyDo(() -> {
                                    double[] positions = drive.getWheelRadiusCharacterizationPositions();
                                    double wheelDelta = 0.0;
                                    for (int i = 0; i < 4; i++) {
                                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                                    }
                                    double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                                    NumberFormat formatter = new DecimalFormat("#0.000");
                                    System.out.println("********** Wheel Radius Characterization Results **********");
                                    System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                                    System.out.println(
                                            "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                                    System.out.println("\tWheel Radius: "
                                            + formatter.format(wheelRadius)
                                            + " meters, "
                                            + formatter.format(Units.metersToInches(wheelRadius))
                                            + " inches");
                                })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }
}
