package org.frc5687.chargedup.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.subsystems.EndEffector.EndEffectorState;
import org.frc5687.chargedup.util.Nodes;
import org.frc5687.chargedup.util.SuperStructureSetpoints;

/**
 * This command tells the arm and drivetrain to go to a specific setpoint and location based on a
 * button press For instance if they press Node 1 High, it sets the setpoint to be arm place high
 * and drivetrain setpoint to node 1
 */
public class SetRobotGoal extends OutliersCommand {
    private final DriveTrain _driveTrain;
    private final EndEffector _endEffector;
    private final Nodes.Node _node;
    private final Nodes.Level _goalLevel;
    private final Translation2d _redOffset = new Translation2d(0.05, 0).times(-1.0);
    private final Translation2d _blueOffset = new Translation2d(0.05,  0);

    public SetRobotGoal(
            DriveTrain driveTrain, EndEffector endEffector, Nodes.Node node, Nodes.Level position) {
        _driveTrain = driveTrain;
        _endEffector = endEffector;
        _node = node;
        _goalLevel = position;
    }

    @Override
    public void initialize() {
        _endEffector.setGoalLevel(_goalLevel);
        super.initialize();
        error("Setting to position " + _goalLevel.name() + "on node " + _node.name());
        boolean isRedAlliance = DriverStation.getAlliance() == DriverStation.Alliance.Red;
        Translation2d offset = isRedAlliance ? _redOffset : _blueOffset;
        Pose2d pose =
                isRedAlliance
                        ? Constants.Auto.FieldPoses.RED_NODE_ONE_GOAL
                        : Constants.Auto.FieldPoses.BLUE_NODE_ONE_GOAL;
        SuperStructureSetpoints.Setpoint setpoint = SuperStructureSetpoints.idleConeSetpoint;
        switch (_node) {
            case ONE:
                pose =
                        isRedAlliance
                                ? Constants.Auto.FieldPoses.RED_NODE_ONE_GOAL
                                : Constants.Auto.FieldPoses.BLUE_NODE_NINE_GOAL;
                switch (_goalLevel) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleConeSetpoint;
                        _endEffector.setState(EndEffectorState.GROUND);
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleConePlaceSetpoint;
                        _endEffector.setState(EndEffectorState.CONE);
                        pose = new Pose2d(pose.getTranslation().plus(offset), 
                            pose.getRotation());
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highConePlaceSetpoint;
                        _endEffector.setState(EndEffectorState.CONE);
                        break;
                }
                break;
            case TWO:
                pose =
                        isRedAlliance
                                ? Constants.Auto.FieldPoses.RED_NODE_TWO_GOAL
                                : Constants.Auto.FieldPoses.BLUE_NODE_EIGHT_GOAL;
                switch (_goalLevel) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleCubeSetpoint;
                        _endEffector.setState(EndEffectorState.GROUND);
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleCubePlaceSetpoint;
                        _endEffector.setState(EndEffectorState.CUBE);
                        pose = new Pose2d(pose.getTranslation().plus(offset), 
                            pose.getRotation());
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highCubePlaceSetpoint;
                        _endEffector.setState(EndEffectorState.CUBE);
                        break;
                }
                break;
            case THREE:
                pose =
                        isRedAlliance
                                ? Constants.Auto.FieldPoses.RED_NODE_THREE_GOAL
                                : Constants.Auto.FieldPoses.BLUE_NODE_SEVEN_GOAL;
                switch (_goalLevel) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleConeSetpoint;
                        _endEffector.setState(EndEffectorState.GROUND);
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleConePlaceSetpoint;
                        _endEffector.setState(EndEffectorState.CONE);
                        pose = new Pose2d(pose.getTranslation().plus(offset), 
                            pose.getRotation());
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highConePlaceSetpoint;
                        _endEffector.setState(EndEffectorState.CONE);
                        break;
                }
                break;
            case FOUR:
                pose =
                        isRedAlliance
                                ? Constants.Auto.FieldPoses.RED_NODE_FOUR_GOAL
                                : Constants.Auto.FieldPoses.BLUE_NODE_SIX_GOAL;
                switch (_goalLevel) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleConeSetpoint;
                        _endEffector.setState(EndEffectorState.GROUND);
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleConePlaceSetpoint;
                        _endEffector.setState(EndEffectorState.CONE);
                        pose = new Pose2d(pose.getTranslation().plus(offset), 
                            pose.getRotation());
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highConePlaceSetpoint;
                        _endEffector.setState(EndEffectorState.CONE);
                        break;
                }
                break;
            case FIVE:
                pose =
                        isRedAlliance
                                ? Constants.Auto.FieldPoses.RED_NODE_FIVE_GOAL
                                : Constants.Auto.FieldPoses.BLUE_NODE_FIVE_GOAL;
                switch (_goalLevel) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleCubeSetpoint;
                        _endEffector.setState(EndEffectorState.GROUND);
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleCubePlaceSetpoint;
                        _endEffector.setState(EndEffectorState.CUBE);
                        pose = new Pose2d(pose.getTranslation().plus(offset), 
                            pose.getRotation());
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highCubePlaceSetpoint;
                        _endEffector.setState(EndEffectorState.CUBE);
                        break;
                }
                break;
            case SIX:
                pose =
                        isRedAlliance
                                ? Constants.Auto.FieldPoses.RED_NODE_SIX_GOAL
                                : Constants.Auto.FieldPoses.BLUE_NODE_FOUR_GOAL;
                switch (_goalLevel) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleConeSetpoint;
                        _endEffector.setState(EndEffectorState.GROUND);
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleConePlaceSetpoint;
                        _endEffector.setState(EndEffectorState.CONE);
                        pose = new Pose2d(pose.getTranslation().plus(offset), 
                            pose.getRotation());
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highConePlaceSetpoint;
                        _endEffector.setState(EndEffectorState.CONE);
                        break;
                }
                break;
            case SEVEN:
                pose =
                        isRedAlliance
                                ? Constants.Auto.FieldPoses.RED_NODE_SEVEN_GOAL
                                : Constants.Auto.FieldPoses.BLUE_NODE_THREE_GOAL;
                switch (_goalLevel) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleConeSetpoint;
                        _endEffector.setState(EndEffectorState.GROUND);
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleConePlaceSetpoint;
                        _endEffector.setState(EndEffectorState.CONE);
                        pose = new Pose2d(pose.getTranslation().plus(offset), 
                            pose.getRotation());
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highConePlaceSetpoint;
                        _endEffector.setState(EndEffectorState.CONE);
                        break;
                }
                break;
            case EIGHT:
                pose =
                        isRedAlliance
                                ? Constants.Auto.FieldPoses.RED_NODE_EIGHT_GOAL
                                : Constants.Auto.FieldPoses.BLUE_NODE_TWO_GOAL;
                switch (_goalLevel) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleCubeSetpoint;
                        _endEffector.setState(EndEffectorState.GROUND);
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleCubePlaceSetpoint;
                        _endEffector.setState(EndEffectorState.CUBE);
                        pose = new Pose2d(pose.getTranslation().plus(offset), 
                            pose.getRotation());
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highCubePlaceSetpoint;
                        _endEffector.setState(EndEffectorState.CUBE);
                        break;
                }
                break;
            case NINE:
                pose =
                        isRedAlliance
                                ? Constants.Auto.FieldPoses.RED_NODE_NINE_GOAL
                                : Constants.Auto.FieldPoses.BLUE_NODE_ONE_GOAL;
                switch (_goalLevel) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleConeSetpoint;
                        _endEffector.setState(EndEffectorState.GROUND);
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleConePlaceSetpoint;
                        _endEffector.setState(EndEffectorState.CONE);
                        pose = new Pose2d(pose.getTranslation().plus(offset), 
                            pose.getRotation());
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highConePlaceSetpoint;
                        _endEffector.setState(EndEffectorState.CONE);
                        break;
                }
                  break;
        }
        _endEffector.setSuperStructureSetpoint(setpoint);
        _driveTrain.setHoverGoal(pose);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
