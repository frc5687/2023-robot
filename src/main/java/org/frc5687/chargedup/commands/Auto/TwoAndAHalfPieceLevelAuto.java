package org.frc5687.chargedup.commands.Auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.*;
import org.frc5687.chargedup.commands.CubeShooter.AutoIntake;
import org.frc5687.chargedup.commands.CubeShooter.Shoot;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.CubeShooter;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.subsystems.Lights;
import org.frc5687.chargedup.util.AutoChooser;
import org.frc5687.chargedup.util.FieldConstants;
import org.frc5687.chargedup.util.Trajectories;
import org.frc5687.chargedup.util.Nodes.Level;
import org.frc5687.chargedup.util.Nodes.Node;

import static org.frc5687.chargedup.util.SuperStructureSetpoints.idleConeSetpoint;

public class TwoAndAHalfPieceLevelAuto extends SequentialCommandGroup {
    private PathPlannerTrajectory _trajectory1;
    private PathPlannerTrajectory _trajectory2;
    private PathPlannerTrajectory _trajectory3;
    private PathPlannerTrajectory _trajectory4;
    private Pose2d pose;
    // private Rotation2d rotation1;
    // private Rotation2d rotation2;

    public TwoAndAHalfPieceLevelAuto(
        DriveTrain driveTrain,
        EndEffector endEffector,
        Elevator elevator,
        Arm arm,
        Lights lights,
        CubeShooter _shooter,
        OI _oi,
        AutoChooser.Node _node,
        Trajectories trajectories
        ) {
        String alliance = driveTrain.isRedAlliance() ? "RED_" : "BLUE_";
        boolean waitInstead = false;
        boolean placeCone = false;

        switch (_node) {
            case OneCone:
                DriverStation.reportError("Unimplemented case: " + _node, false);
                waitInstead = true;
                break;
            case TwoCube:
                DriverStation.reportError("Unimplemented case: " + _node, false);
                waitInstead = true;
                break;
            case ThreeCone:
                DriverStation.reportError("Unimplemented case: " + _node, false);
                waitInstead = true;
                break;
            case FourCone:
                DriverStation.reportError("Unimplemented case: " + _node, false);
                waitInstead = true;
                break;
            case FiveCube:
                DriverStation.reportError("Unimplemented case: " + _node, false);
                waitInstead = true;
                break;
            case SixCone:
                DriverStation.reportError("Unimplemented case: " + _node, false);
                waitInstead = true;
                break;
            case SevenCone:
                DriverStation.reportError("Unimplemented case: " + _node, false);
                waitInstead = true;
                break;
            case EightCube:
                _trajectory1 = trajectories.getTrajectory(alliance + "NODE_EIGHT_GOAL_FOUR");
                _trajectory2 = trajectories.getTrajectory(alliance + "GOAL_FOUR_NODE_EIGHT");
                _trajectory3 = trajectories.getTrajectory(alliance + "NODE_EIGHT_SHOOT_GOAL_THREE");
                _trajectory4 = trajectories.getTrajectory(alliance + "NOBUMP_GOAL_THREE_CHARGE_FOUR");
                pose = driveTrain.isRedAlliance() ? Constants.Auto.FieldPoses.RED_NODE_EIGHT_GOAL : Constants.Auto.FieldPoses.BLUE_NODE_EIGHT_GOAL;
                placeCone = false;
                break;
            case NineCone:
                _trajectory1 = trajectories.getTrajectory(alliance + "NODE_NINE_GOAL_FOUR");
                _trajectory2 = trajectories.getTrajectory(alliance + "GOAL_FOUR_NODE_EIGHT");
                _trajectory3 = trajectories.getTrajectory(alliance + "NODE_EIGHT_SHOOT_GOAL_THREE");
                _trajectory4 = trajectories.getTrajectory(alliance + "NOBUMP_GOAL_THREE_CHARGE_FOUR");
                pose = driveTrain.isRedAlliance() ? Constants.Auto.FieldPoses.RED_NODE_EIGHT_GOAL : Constants.Auto.FieldPoses.BLUE_NODE_EIGHT_GOAL;
                placeCone = true;
                break;
            case Unknown:
                DriverStation.reportError("Unimplemented case: " + _node, false);
                waitInstead = true;
                break;
            default:
                DriverStation.reportError("Unexpected value: " + _node, false);
                waitInstead = true;
                break;
        }
        Command placeCommand = placeCone ? new AutoPlaceHighCone(elevator, endEffector, arm) : new AutoPlaceHighCube(elevator, endEffector, arm);
        if (waitInstead) {
            addCommands(new WaitCommand(15));
        } else {
            addCommands(
                new SequentialCommandGroup(
                    new ResetRobotPose(driveTrain, _trajectory1.getInitialHolonomicPose()),
                    placeCommand,
                    new ParallelDeadlineGroup(
                        new DriveTrajectory(driveTrain, _trajectory1, true, false),
                            new AutoSetSuperStructurePosition(elevator, endEffector, arm, idleConeSetpoint),
                            new AutoIntake(_shooter, true, _oi)
                    ),
                    new DriveTrajectory(driveTrain, _trajectory2, true, false),
                    new DriveToPose(driveTrain, pose.transformBy(new Transform2d(new Translation2d(0.1, 0), new Rotation2d())), true),
                    new Shoot(_shooter, 1.0, Constants.CubeShooter.IDLE_ANGLE),
                    new ParallelDeadlineGroup(
                        new DriveTrajectory(driveTrain, _trajectory3, true, false),
                        new AutoIntake(_shooter, true, _oi)
                        ),
                    new DriveTrajectory(driveTrain, _trajectory4, true, false),
                    new QuickLevel(driveTrain)
                )
            );
        }
    }
}
