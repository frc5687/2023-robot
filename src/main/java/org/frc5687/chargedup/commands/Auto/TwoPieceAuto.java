package org.frc5687.chargedup.commands.Auto;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.DriveTrajectory;
import org.frc5687.chargedup.commands.CubeShooter.AutoIntake;
import org.frc5687.chargedup.commands.CubeShooter.Shoot;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.CubeShooter;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.subsystems.Lights;
import org.frc5687.chargedup.util.AutoChooser;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TwoPieceAuto extends SequentialCommandGroup {
    private Trajectory _trajectory1;
    private Trajectory _trajectory2;
    //private Rotation2d rotation1;
    //private Rotation2d rotation2;
    

   
    public TwoPieceAuto(
        DriveTrain driveTrain,
        EndEffector endEffector,
        Elevator elevator,
        Arm arm,
        Lights lights,
        CubeShooter _shooter,
        OI _oi, 
        AutoChooser.Node _node
    ){
         var config = driveTrain.getConfig();
          switch(_node) {
            case OneCone:
                if (DriverStation.getAlliance() == Alliance.Red){
                    driveTrain.resetRobotPose(Constants.Auto.FieldPoses.RED_NODE_ONE_GOAL);
                    _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node1.RED_NODE_ONE_TRAJECTORY_ONE, config);
                    _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node1.RED_NODE_ONE_TRAJECTORY_TWO, config);
            } else { 
                    driveTrain.resetRobotPose(Constants.Auto.FieldPoses.BLUE_NODE_ONE_GOAL);
                    _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node1.BLUE_NODE_ONE_TRAJECTORY_ONE, config);
                    _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node1.BLUE_NODE_ONE_TRAJECTORY_TWO, config);
                }
                break;
            case TwoCube:
                if (DriverStation.getAlliance() == Alliance.Red){
                    driveTrain.resetRobotPose(Constants.Auto.FieldPoses.RED_NODE_TWO_GOAL);
                    _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node2.RED_NODE_TWO_TRAJECTORY_ONE, config);
                    _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node2.RED_NODE_TWO_TRAJECTORY_TWO, config);
            } else {
                    driveTrain.resetRobotPose(Constants.Auto.FieldPoses.BLUE_NODE_TWO_GOAL);
                    _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node2.BLUE_NODE_TWO_TRAJECTORY_ONE, config);
                    _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node2.BLUE_NODE_TWO_TRAJECTORY_TWO, config);
                }
                break;
            case ThreeCone:
                if (DriverStation.getAlliance() == Alliance.Red){
                    driveTrain.resetRobotPose(Constants.Auto.FieldPoses.RED_NODE_THREE_GOAL);
                    _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node3.RED_NODE_THREE_TRAJECTORY_ONE, config);
                    _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node3.RED_NODE_THREE_TRAJECTORY_TWO, config);
            } else {
                    driveTrain.resetRobotPose(Constants.Auto.FieldPoses.BLUE_NODE_THREE_GOAL);
                    _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node3.BLUE_NODE_THREE_TRAJECTORY_ONE, config);
                    _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node3.BLUE_NODE_THREE_TRAJECTORY_TWO, config);
                }
                break;
            case FourCone:
                if (DriverStation.getAlliance() == Alliance.Red){
                    driveTrain.resetRobotPose(Constants.Auto.FieldPoses.RED_NODE_FOUR_GOAL);
                    _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node4.RED_NODE_FOUR_TRAJECTORY_ONE, config);
                    _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node4.RED_NODE_FOUR_TRAJECTORY_TWO, config);
            } else {
                    driveTrain.resetRobotPose(Constants.Auto.FieldPoses.BLUE_NODE_FOUR_GOAL);
                    _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node4.BLUE_NODE_FOUR_TRAJECTORY_ONE, config);
                    _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node4.BLUE_NODE_FOUR_TRAJECTORY_TWO, config);
                }
                break;
            case FiveCube:
                if (DriverStation.getAlliance() == Alliance.Red){
                    driveTrain.resetRobotPose(Constants.Auto.FieldPoses.RED_NODE_FIVE_GOAL);
                    _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node5.RED_NODE_FIVE_TRAJECTORY_ONE, config);
                    _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node5.RED_NODE_FIVE_TRAJECTORY_TWO, config);
            } else{
                    driveTrain.resetRobotPose(Constants.Auto.FieldPoses.BLUE_NODE_FIVE_GOAL);
                    _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node5.BLUE_NODE_FIVE_TRAJECTORY_ONE, config);
                    _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node5.BLUE_NODE_FIVE_TRAJECTORY_TWO, config);
                }
                break;
            case SixCone:
                if (DriverStation.getAlliance() == Alliance.Red){
                    driveTrain.resetRobotPose(Constants.Auto.FieldPoses.RED_NODE_SIX_GOAL);
                    _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node6.RED_NODE_SIX_TRAJECTORY_ONE, config);
                    _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node6.RED_NODE_SIX_TRAJECTORY_TWO, config);
                } else {
                    driveTrain.resetRobotPose(Constants.Auto.FieldPoses.BLUE_NODE_SIX_GOAL);
                    _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node6.BLUE_NODE_SIX_TRAJECTORY_ONE, config);
                    _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node6.BLUE_NODE_SIX_TRAJECTORY_TWO, config);
                }
                break;
            case SevenCone:
                if (DriverStation.getAlliance() == Alliance.Red){
                    driveTrain.resetRobotPose(Constants.Auto.FieldPoses.RED_NODE_SEVEN_GOAL);
                    _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node7.RED_NODE_SEVEN_TRAJECTORY_ONE, config);
                    _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node7.RED_NODE_SEVEN_TRAJECTORY_TWO, config);
                } else {
                    driveTrain.resetRobotPose(Constants.Auto.FieldPoses.BLUE_NODE_SEVEN_GOAL);
                    _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node7.BLUE_NODE_SEVEN_TRAJECTORY_ONE, config);
                    _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node7.BLUE_NODE_SEVEN_TRAJECTORY_TWO, config);
                }    
                break;
            case EightCube:
                if (DriverStation.getAlliance() == Alliance.Red){
                    driveTrain.resetRobotPose(Constants.Auto.FieldPoses.RED_NODE_EIGHT_GOAL);
                    _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node8.RED_NODE_EIGHT_TRAJECTORY_ONE, config);
                    _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node8.RED_NODE_EIGHT_TRAJECTORY_TWO, config);
                } else {
                    driveTrain.resetRobotPose(Constants.Auto.FieldPoses.BLUE_NODE_EIGHT_GOAL);
                    _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node8.BLUE_NODE_EIGHT_TRAJECTORY_ONE, config);
                    _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node8.BLUE_NODE_EIGHT_TRAJECTORY_TWO, config);
                }
                break;
            case NineCone:
                if (DriverStation.getAlliance() == Alliance.Red){
                    driveTrain.resetRobotPose(Constants.Auto.FieldPoses.RED_NODE_NINE_GOAL);
                    _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node9.RED_NODE_NINE_TRAJECTORY_ONE, config);
                    _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node9.RED_NODE_NINE_TRAJECTORY_TWO, config);
                } else {
                    driveTrain.resetRobotPose(Constants.Auto.FieldPoses.BLUE_NODE_NINE_GOAL);
                    _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node9.BLUE_NODE_NINE_TRAJECTORY_ONE, config);
                    _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node9.BLUE_NODE_NINE_TRAJECTORY_TWO, config);
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
                    new ParallelCommandGroup(
                        new DriveTrajectory(driveTrain, _trajectory1),
                        new AutoIntake(_shooter)
                    ),   
                new DriveTrajectory(driveTrain, _trajectory2),
                new Shoot(_shooter, Constants.CubeShooter.SHOOT_RPS, Constants.CubeShooter.IDLE_ANGLE)
            )
        );
    }
}