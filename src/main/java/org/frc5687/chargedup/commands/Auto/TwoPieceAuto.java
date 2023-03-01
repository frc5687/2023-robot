package org.frc5687.chargedup.commands.Auto;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.DriveTrajectory;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.subsystems.Lights;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
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
        Lights lights 
    ){
        /* var config = driveTrain.getConfig();
         switch(Node) {
            case Node.1cone:
                _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node1.NODE_ONE_TRAJECTORY_ONE, config);
                _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node1.NODE_ONE_TRAJECTORY_TWO, config);
                break;
            case Node.2cube:
               _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node2.NODE_TWO_TRAJECTORY_ONE, config);
               _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node2.NODE_TWO_TRAJECTORY_TWO, config);
                break;
            case Node.3cone:
                _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node3.NODE_THREE_TRAJECTORY_ONE, config);
                _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node3.NODE_THREE_TRAJECTORY_TWO, config);
                break;
            case Node.4cone:
                _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node4.NODE_FOUR_TRAJECTORY_ONE, config);
                _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node4.NODE_FOUR_TRAJECTORY_TWO, config);
                break;
            case Node.5cube:
                _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node5.NODE_FIVE_TRAJECTORY_ONE, config);
                _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node5.NODE_FIVE_TRAJECTORY_TWO, config);
                break;
            case Node.6cone:
                _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node6.NODE_SIX_TRAJECTORY_ONE, config);
                _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node6.NODE_SIX_TRAJECTORY_TWO, config);
                break;
            case Node.7cone:
                _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node7.NODE_SEVEN_TRAJECTORY_ONE, config);
                _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node7.NODE_SEVEN_TRAJECTORY_TWO, config);
                break;
            case Node.8cube:
                _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node8.NODE_EIGHT_TRAJECTORY_ONE, config);
                _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node8.NODE_EIGHT_TRAJECTORY_TWO, config);
                break;
            case Node.9cone:
                _trajectory1 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node9.NODE_NINE_TRAJECTORY_ONE, config);
                _trajectory2 = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.Node9.NODE_NINE_TRAJECTORY_TWO, config);
                break; 
        } */
                   
        

    
        addCommands(
            new SequentialCommandGroup(
                new AutoPlaceHighCube(arm, endEffector, elevator),
                new DriveTrajectory(driveTrain, _trajectory1),
                new AutoGroundPickupCube(elevator, arm, endEffector),
                new DriveTrajectory(driveTrain, _trajectory2),
                new AutoPlaceHighCone(elevator, endEffector, arm)
            )
        );
        
    }
}