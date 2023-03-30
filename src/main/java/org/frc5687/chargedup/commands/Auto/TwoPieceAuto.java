package org.frc5687.chargedup.commands.Auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.CubeShooter.AutoIntake;
import org.frc5687.chargedup.commands.CubeShooter.Shoot;
import org.frc5687.chargedup.commands.DriveTrajectory;
import org.frc5687.chargedup.commands.SetHoverGoal;
import org.frc5687.chargedup.commands.SetRobotGoal;
import org.frc5687.chargedup.commands.SnapTo;
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
        Trajectories trajectories
        ) {
        String alliance = driveTrain.isRedAlliance() ? "RED_" : "BLUE_";
        boolean waitInstead = false;
        boolean placeCone = false;

        switch (_node) {
            case OneCone:
                _trajectory1 = trajectories.getTrajectory(alliance + "NODE_ONE_GOAL_ONE");
                _trajectory2 = trajectories.getTrajectory(alliance + "GOAL_ONE_NODE_TWO");
                placeCone = true;
                break;
            case TwoCube:
                _trajectory1 = trajectories.getTrajectory(alliance + "NODE_TWO_GOAL_ONE");
                _trajectory2 = trajectories.getTrajectory(alliance + "GOAL_ONE_NODE_TWO");
                placeCone = false;
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
                placeCone = false;
                break;
            case NineCone:
                _trajectory1 = trajectories.getTrajectory(alliance + "NODE_NINE_GOAL_FOUR");
                _trajectory2 = trajectories.getTrajectory(alliance + "GOAL_FOUR_NODE_EIGHT");
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

        if (waitInstead) {
            addCommands(new WaitCommand(15));
        } else {
            if (placeCone) {
                addCommands(
                    new SequentialCommandGroup(
                        new AutoPlaceHighCone(elevator, endEffector, arm),
                        // new DriveForTime(driveTrain, 100),
                        new ParallelDeadlineGroup(
                            new DriveTrajectory(driveTrain, _trajectory1, true, true),
                            new SequentialCommandGroup(
                                new WaitCommand(1), 
                                new AutoIntake(_shooter))
                        ),
                        new DriveTrajectory(driveTrain, _trajectory2, true, false),
                        new DriveToPose(driveTrain, new Pose2d(Constants.Auto.FieldPoses.RED_NODE_TWO_GOAL.getTranslation(), new Rotation2d(Math.PI)), true),
                        new Shoot(_shooter, 0.6, 0.21, _oi)
                    )
                );
            } else {
                addCommands(
                    new SequentialCommandGroup(
                        new AutoPlaceHighCube(elevator, endEffector, arm),
                        new ParallelDeadlineGroup(
                            new DriveTrajectory(driveTrain, _trajectory1, true, true),
                            new AutoIntake(_shooter)
                        ),
                        new DriveTrajectory(driveTrain, _trajectory2, true, false),
                        new SnapTo(driveTrain, new Rotation2d(195)),
                        new Shoot(_shooter, 0.6, 0.21, _oi)
                    )
                );
            }
        }
    }
}
