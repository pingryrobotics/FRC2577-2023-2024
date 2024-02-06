// package frc.robot.commands.drive_commands;

// // Copyright (c) 2023 FRC 6328
// // http://github.com/Mechanical-Advantage
// //
// // Use of this source code is governed by an MIT-style
// // license that can be found in the LICENSE file at
// // the root directory of this project.

// // commented because i don't like seeing yellow dots on my sidebar - christian
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// //import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
// import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
// import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
// //import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import java.util.List;
// import java.util.function.Supplier;
// import frc.robot.Constants;
// import frc.robot.subsystems.Drive;
// import frc.robot.util.trajectory.CustomHolonomicDriveController;
// import frc.robot.util.trajectory.CustomTrajectoryGenerator;
// import frc.robot.util.trajectory.RotationSequence;
// import frc.robot.util.trajectory.Waypoint;
// import frc.robot.util.AllianceFlipUtil;

// public class DriveTrajectory extends Command {
//   private static boolean supportedRobot = true;
//   private static double maxVelocityMetersPerSec = Constants.AutoConstants.kMaxSpeedMetersPerSecond;
//   private static double maxAccelerationMetersPerSec2 = Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared;
//   private static double maxCentripetalAccelerationMetersPerSec2 = Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared;

//   private final PIDController xController = new PIDController(0.0, 0.0, 0.0);
//   private final PIDController yController = new PIDController(0.0, 0.0, 0.0);
//   private final PIDController thetaController = new PIDController(0.0, 0.0, 0.0);

//   private final CustomHolonomicDriveController customHolonomicDriveController =
//       new CustomHolonomicDriveController(xController, yController, thetaController);

//   private final Drive drive;
//   private final Timer timer = new Timer();

//   private Supplier<List<Waypoint>> waypointsSupplier = null;
//   private Supplier<List<TrajectoryConstraint>> constraintsSupplier = null;
//   private CustomTrajectoryGenerator customGenerator = new CustomTrajectoryGenerator();

//   /** Creates a DriveTrajectory command with a dynamic set of waypoints. */
//   public DriveTrajectory(Drive drive, Supplier<List<Waypoint>> waypointsSupplier) {
//     this(drive, waypointsSupplier, () -> List.of());
//   }

//   /** Creates a DriveTrajectory command with a dynamic set of waypoints and constraints. */
//   public DriveTrajectory(
//       Drive drive,
//       Supplier<List<Waypoint>> waypointsSupplier,
//       Supplier<List<TrajectoryConstraint>> constraintsSupplier) {
//     this.drive = drive;
//     addRequirements(drive);
//     this.waypointsSupplier = waypointsSupplier;
//     this.constraintsSupplier = constraintsSupplier;
//   }

//   /** Creates a DriveTrajectory command with a static set of waypoints. */
//   public DriveTrajectory(Drive drive, List<Waypoint> waypoints) {
//     this(drive, waypoints, List.of());
//   }

//   /** Creates a DriveTrajectory command with a static set of waypoints and constraints. */
//   public DriveTrajectory(
//       Drive drive, List<Waypoint> waypoints, List<TrajectoryConstraint> constraints) {
//     this.drive = drive;
//     addRequirements(drive);
//     generate(waypoints, constraints, true);
//   }

//   /** Generates the trajectory. */
//   private void generate(
//       List<Waypoint> waypoints, List<TrajectoryConstraint> constraints, boolean alertOnFail) {
//     // Set up trajectory configuration
//     TrajectoryConfig config =
//         new TrajectoryConfig(maxVelocityMetersPerSec, maxAccelerationMetersPerSec2)
//             .setKinematics(Constants.DriveConstants.kDriveKinematics)
//             .setStartVelocity(0.0)
//             .setEndVelocity(0.0)
//             .addConstraint(
//                 new CentripetalAccelerationConstraint(maxCentripetalAccelerationMetersPerSec2))
//             .addConstraints(constraints);

//     // Generate trajectory
//     customGenerator = new CustomTrajectoryGenerator(); // Reset generator
//     try {
//       customGenerator.generate(config, waypoints);
//     } catch (TrajectoryGenerationException exception) {
//       if (supportedRobot && alertOnFail) {
//         DriverStation.reportError("Failed to generate trajectory.", true);
//       }
//     }
//   }

//   @Override
//   public void initialize() {
//     // Generate trajectory if supplied
//     if (waypointsSupplier != null || constraintsSupplier != null) {
//       generate(waypointsSupplier.get(), constraintsSupplier.get(), false);
//     }

//     // Reset all controllers
//     timer.reset();
//     timer.start();
//     xController.reset();
//     yController.reset();
//     thetaController.reset();

//     // Reset PID gains
//     xController.setP(Constants.ModuleConstants.kDrivingP);
//     xController.setD(Constants.ModuleConstants.kDrivingD);
//     yController.setP(Constants.ModuleConstants.kDrivingP);
//     yController.setD(Constants.ModuleConstants.kDrivingD);
//     thetaController.setP(Constants.ModuleConstants.kTurningP);
//     thetaController.setD(Constants.ModuleConstants.kTurningD);
//   }

//   @Override
//   public void execute() {
//     // // Update from tunable numbers
//     // if (driveKd.hasChanged(hashCode())
//     //     || driveKp.hasChanged(hashCode())
//     //     || turnKd.hasChanged(hashCode())
//     //     || turnKp.hasChanged(hashCode())) {
//     //   xController.setP(driveKp.get());
//     //   xController.setD(driveKd.get());
//     //   yController.setP(driveKp.get());
//     //   yController.setD(driveKd.get());
//     //   thetaController.setP(turnKp.get());
//     //   thetaController.setD(turnKd.get());
//     // }

//     // Exit if trajectory generation failed
//     if (customGenerator.getDriveTrajectory().getStates().size() <= 1) {
//       return;
//     }

//     // Get setpoint
//     Trajectory.State driveState =
//         AllianceFlipUtil.apply(customGenerator.getDriveTrajectory().sample(timer.get()));
//     RotationSequence.State holonomicRotationState =
//         AllianceFlipUtil.apply(customGenerator.getHolonomicRotationSequence().sample(timer.get()));

//     // Calculate velocity
//     ChassisSpeeds nextDriveState =
//         customHolonomicDriveController.calculate(
//             drive.getPose(), driveState, holonomicRotationState);
//     drive.setModuleStates(drive.kinematics.toSwerveModuleStates(nextDriveState));
//   }

//   @Override
//   public void end(boolean interrupted) {
//     drive.stop();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return timer.hasElapsed(customGenerator.getDriveTrajectory().getTotalTimeSeconds());
//   }
// }