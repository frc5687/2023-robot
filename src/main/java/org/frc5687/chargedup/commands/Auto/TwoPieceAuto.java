package org.frc5687.chargedup.commands.Auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.CubeShooter.AutoIntake;
import org.frc5687.chargedup.commands.CubeShooter.Shoot;
import org.frc5687.chargedup.commands.DriveTrajectory;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.CubeShooter;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.subsystems.Lights;
import org.frc5687.chargedup.util.AutoChooser;
import org.frc5687.chargedup.util.Trajectories;

public class TwoPieceAuto extends SequentialCommandGroup {
    private PathPlannerTrajectory _trajectory1;
    private PathPlannerTrajectory _trajectory2;
    // private Rotation2d rotation1;
    // private Rotation2d rotation2;

    public TwoPieceAuto(
            DriveTrain driveTrain,
            EndEffector endEffector,
            Elevator elevator,
            Arm arm,
            Lights lights,
            CubeShooter _shooter,
            OI _oi,
            AutoChooser.Node _node,
            Trajectories trajectories) {
        //var config = driveTrain.getConfig();
        // _trajectory1 = trajectories.getTrajectory("NODE_ONE_GOAL_ONE");
        // _trajectory2 = trajectories.getTrajectory("NODE_ONE_GOAL_ONE");

       switch (_node) {
           case OneCone:
               if (DriverStation.getAlliance() == Alliance.Red) {
                   driveTrain.resetRobotPose(Constants.Auto.FieldPoses.RED_NODE_ONE_GOAL);
                   _trajectory1 = trajectories.getTrajectory("NODE_ONE_GOAL_ONE");
                   _trajectory2 = trajectories.getTrajectory("GOAL_ONE_NODE_TWO");
               } else {
                   driveTrain.resetRobotPose(Constants.Auto.FieldPoses.BLUE_NODE_ONE_GOAL);
                   _trajectory1 = trajectories.getTrajectory("NODE_ONE_GOAL)ONE");
                   _trajectory2 = trajectories.getTrajectory("GOAL_ONE_NODE_TWO");
               }
               break;
           case TwoCube:
               if (DriverStation.getAlliance() == Alliance.Red) {
                   driveTrain.resetRobotPose(Constants.Auto.FieldPoses.RED_NODE_TWO_GOAL);
                   _trajectory1 = trajectories.getTrajectory("NODE_TWO_GOAL_ONE");
                   _trajectory2 = trajectories.getTrajectory("GOAL_ONE_NODE_ONE");
               } else {
                   driveTrain.resetRobotPose(Constants.Auto.FieldPoses.BLUE_NODE_TWO_GOAL);
                   _trajectory1 = trajectories.getTrajectory("NODE_TWO_GOAL_ONE");
                   _trajectory2 = trajectories.getTrajectory("GOAL_ONE_NODE_ONE");
               break;
               }
           /* case ThreeCone:
               if (DriverStation.getAlliance() == Alliance.Red) {
                   driveTrain.resetRobotPose(Constants.Auto.FieldPoses.RED_NODE_THREE_GOAL);
                   _trajectory1 = trajectories.getTrajectory("NODE_THREE_GOAL_ONE");
                   _trajectory2 = trajectories.getTrajectory("GOAL_ONE_NODE_TWO");
               } else {
                   driveTrain.resetRobotPose(Constants.Auto.FieldPoses.BLUE_NODE_THREE_GOAL);
                   _trajectory1 =
                           TrajectoryGenerator.generateTrajectory(
                                   Constants.Auto.TrajectoryPoints.Node3.BLUE_NODE_THREE_TRAJECTORY_ONE, config);
                   _trajectory2 =
                           TrajectoryGenerator.generateTrajectory(
                                   Constants.Auto.TrajectoryPoints.Node3.BLUE_NODE_THREE_TRAJECTORY_TWO, config);
               }
               break;
           case FourCone:
               if (DriverStation.getAlliance() == Alliance.Red) {
                   driveTrain.resetRobotPose(Constants.Auto.FieldPoses.RED_NODE_FOUR_GOAL);
                   _trajectory1 =
                           TrajectoryGenerator.generateTrajectory(
                                   Constants.Auto.TrajectoryPoints.Node4.RED_NODE_FOUR_TRAJECTORY_ONE, config);
                   _trajectory2 =
                           TrajectoryGenerator.generateTrajectory(
                                   Constants.Auto.TrajectoryPoints.Node4.RED_NODE_FOUR_TRAJECTORY_TWO, config);
               } else {
                   driveTrain.resetRobotPose(Constants.Auto.FieldPoses.BLUE_NODE_FOUR_GOAL);
                   _trajectory1 =
                           TrajectoryGenerator.generateTrajectory(
                                   Constants.Auto.TrajectoryPoints.Node4.BLUE_NODE_FOUR_TRAJECTORY_ONE, config);
                   _trajectory2 =
                           TrajectoryGenerator.generateTrajectory(
                                   Constants.Auto.TrajectoryPoints.Node4.BLUE_NODE_FOUR_TRAJECTORY_TWO, config);
               }
               break;
           case FiveCube:
               if (DriverStation.getAlliance() == Alliance.Red) {
                   driveTrain.resetRobotPose(Constants.Auto.FieldPoses.RED_NODE_FIVE_GOAL);
                   _trajectory1 =
                           TrajectoryGenerator.generateTrajectory(
                                   Constants.Auto.TrajectoryPoints.Node5.RED_NODE_FIVE_TRAJECTORY_ONE, config);
                   _trajectory2 =
                           TrajectoryGenerator.generateTrajectory(
                                   Constants.Auto.TrajectoryPoints.Node5.RED_NODE_FIVE_TRAJECTORY_TWO, config);
               } else {
                   driveTrain.resetRobotPose(Constants.Auto.FieldPoses.BLUE_NODE_FIVE_GOAL);
                   _trajectory1 =
                           TrajectoryGenerator.generateTrajectory(
                                   Constants.Auto.TrajectoryPoints.Node5.BLUE_NODE_FIVE_TRAJECTORY_ONE, config);
                   _trajectory2 =
                           TrajectoryGenerator.generateTrajectory(
                                   Constants.Auto.TrajectoryPoints.Node5.BLUE_NODE_FIVE_TRAJECTORY_TWO, config);
               }
               break;
           case SixCone:
               if (DriverStation.getAlliance() == Alliance.Red) {
                   driveTrain.resetRobotPose(Constants.Auto.FieldPoses.RED_NODE_SIX_GOAL);
                   _trajectory1 =
                           TrajectoryGenerator.generateTrajectory(
                                   Constants.Auto.TrajectoryPoints.Node6.RED_NODE_SIX_TRAJECTORY_ONE, config);
                   _trajectory2 =
                           TrajectoryGenerator.generateTrajectory(
                                   Constants.Auto.TrajectoryPoints.Node6.RED_NODE_SIX_TRAJECTORY_TWO, config);
               } else {
                   driveTrain.resetRobotPose(Constants.Auto.FieldPoses.BLUE_NODE_SIX_GOAL);
                   _trajectory1 =
                           TrajectoryGenerator.generateTrajectory(
                                   Constants.Auto.TrajectoryPoints.Node6.BLUE_NODE_SIX_TRAJECTORY_ONE, config);
                   _trajectory2 =
                           TrajectoryGenerator.generateTrajectory(
                                   Constants.Auto.TrajectoryPoints.Node6.BLUE_NODE_SIX_TRAJECTORY_TWO, config);
               }
               break;
           case SevenCone:
               if (DriverStation.getAlliance() == Alliance.Red) {
                   driveTrain.resetRobotPose(Constants.Auto.FieldPoses.RED_NODE_SEVEN_GOAL);
                   _trajectory1 =
                           TrajectoryGenerator.generateTrajectory(
                                   Constants.Auto.TrajectoryPoints.Node7.RED_NODE_SEVEN_TRAJECTORY_ONE, config);
                   _trajectory2 =
                           TrajectoryGenerator.generateTrajectory(
                                   Constants.Auto.TrajectoryPoints.Node7.RED_NODE_SEVEN_TRAJECTORY_TWO, config);
               } else {
                   driveTrain.resetRobotPose(Constants.Auto.FieldPoses.BLUE_NODE_SEVEN_GOAL);
                   _trajectory1 =
                           TrajectoryGenerator.generateTrajectory(
                                   Constants.Auto.TrajectoryPoints.Node7.BLUE_NODE_SEVEN_TRAJECTORY_ONE, config);
                   _trajectory2 =
                           TrajectoryGenerator.generateTrajectory(
                                   Constants.Auto.TrajectoryPoints.Node7.BLUE_NODE_SEVEN_TRAJECTORY_TWO, config);
               }
               break; */
           case EightCube:
               if (DriverStation.getAlliance() == Alliance.Red) {
                   driveTrain.resetRobotPose(Constants.Auto.FieldPoses.RED_NODE_EIGHT_GOAL);
                  _trajectory1 = trajectories.getTrajectory("NODE_EIGHT_GOAL_FOUR");
                  _trajectory2 = trajectories.getTrajectory("GOAL_FOUR_NODE_NINE");
               } else {
                   driveTrain.resetRobotPose(Constants.Auto.FieldPoses.BLUE_NODE_EIGHT_GOAL);
                   _trajectory1 = trajectories.getTrajectory("NODE_EIGHT_GOAL_FOUR");
                   _trajectory2 = trajectories.getTrajectory("GOAL_FOUR_NODE_NINE");
               break;
               }
           case NineCone:
               if (DriverStation.getAlliance() == Alliance.Red) {
                   driveTrain.resetRobotPose(Constants.Auto.FieldPoses.RED_NODE_NINE_GOAL);
                   _trajectory1 = trajectories.getTrajectory("NODE_NINE_GOAL_EIGHT");
                   _trajectory2 = trajectories.getTrajectory("GOAL_FOUR_NODE_EIGHT");
               } else {
                   driveTrain.resetRobotPose(Constants.Auto.FieldPoses.BLUE_NODE_NINE_GOAL);
                   _trajectory1 = trajectories.getTrajectory("NODE_NINE_GOAL_EIGHT");
                   _trajectory2 = trajectories.getTrajectory("GOAL_FOUR_NODE_EIGHT");
               }
               break;
           case Unknown:
               throw new UnsupportedOperationException("Unimplemented case: " + _node);
           default:
               throw new IllegalArgumentException("Unexpected value: " + _node);
       }

        addCommands(
            new SequentialCommandGroup(
                new AutoPlaceHighCube(elevator, endEffector, arm),
                    new ParallelDeadlineGroup(
                        new DriveTrajectory(driveTrain, _trajectory1, true),
                        new AutoIntake(_shooter)
                    ),   
                new DriveTrajectory(driveTrain, _trajectory2, true),
                new Shoot(_shooter, 0.6, 0.21, _oi)
            )
        );
    }
}
