package org.frc5687.chargedup.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.subsystems.EndEffector.IntakeState;
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
        Pose2d pose =
                isRedAlliance
                        ? Constants.Auto.FieldPoses.RED_NODE_ONE_GOAL
                        : Constants.Auto.FieldPoses.BLUE_NODE_ONE_GOAL;
        SuperStructureSetpoints.Setpoint setpoint = SuperStructureSetpoints.idleConeSetpoint;
        switch (_node) {
            case ONE:
                switch (_goalLevel) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleConeSetpoint;
                        _endEffector.setState(IntakeState.GROUND);
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleConePlaceSetpoint;
                        _endEffector.setState(IntakeState.CONE);
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highConePlaceSetpoint;
                        _endEffector.setState(IntakeState.CONE);
                        break;
                }
                pose =
                        isRedAlliance
                                ? Constants.Auto.FieldPoses.RED_NODE_ONE_GOAL
                                : Constants.Auto.FieldPoses.BLUE_NODE_NINE_GOAL;
                break;
            case TWO:
                switch (_goalLevel) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleCubeSetpoint;
                        _endEffector.setState(IntakeState.GROUND);
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleCubePlaceSetpoint;
                        _endEffector.setState(IntakeState.CUBE);
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highCubePlaceSetpoint;
                        _endEffector.setState(IntakeState.CUBE);
                        break;
                }
                pose =
                        isRedAlliance
                                ? Constants.Auto.FieldPoses.RED_NODE_TWO_GOAL
                                : Constants.Auto.FieldPoses.BLUE_NODE_EIGHT_GOAL;
                break;
            case THREE:
                switch (_goalLevel) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleConeSetpoint;
                        _endEffector.setState(IntakeState.GROUND);
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleConePlaceSetpoint;
                        _endEffector.setState(IntakeState.CONE);
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highConePlaceSetpoint;
                        _endEffector.setState(IntakeState.CONE);
                        break;
                }
                pose =
                        isRedAlliance
                                ? Constants.Auto.FieldPoses.RED_NODE_THREE_GOAL
                                : Constants.Auto.FieldPoses.BLUE_NODE_SEVEN_GOAL;
                break;
            case FOUR:
                switch (_goalLevel) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleConeSetpoint;
                        _endEffector.setState(IntakeState.GROUND);
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleConePlaceSetpoint;
                        _endEffector.setState(IntakeState.CONE);
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highConePlaceSetpoint;
                        _endEffector.setState(IntakeState.CONE);
                        break;
                }
                pose =
                        isRedAlliance
                                ? Constants.Auto.FieldPoses.RED_NODE_FOUR_GOAL
                                : Constants.Auto.FieldPoses.BLUE_NODE_SIX_GOAL;
                break;
            case FIVE:
                switch (_goalLevel) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleCubeSetpoint;
                        _endEffector.setState(IntakeState.GROUND);
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleCubePlaceSetpoint;
                        _endEffector.setState(IntakeState.CUBE);
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highCubePlaceSetpoint;
                        _endEffector.setState(IntakeState.CUBE);
                        break;
                }
                pose =
                        isRedAlliance
                                ? Constants.Auto.FieldPoses.RED_NODE_FIVE_GOAL
                                : Constants.Auto.FieldPoses.BLUE_NODE_FIVE_GOAL;
                break;
            case SIX:
                switch (_goalLevel) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleConeSetpoint;
                        _endEffector.setState(IntakeState.GROUND);
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleConePlaceSetpoint;
                        _endEffector.setState(IntakeState.CONE);
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highConePlaceSetpoint;
                        _endEffector.setState(IntakeState.CONE);
                        break;
                }
                pose =
                        isRedAlliance
                                ? Constants.Auto.FieldPoses.RED_NODE_SIX_GOAL
                                : Constants.Auto.FieldPoses.BLUE_NODE_FOUR_GOAL;
                break;
            case SEVEN:
                switch (_goalLevel) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleConeSetpoint;
                        _endEffector.setState(IntakeState.GROUND);
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleConePlaceSetpoint;
                        _endEffector.setState(IntakeState.CONE);
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highConePlaceSetpoint;
                        _endEffector.setState(IntakeState.CONE);
                        break;
                }
                pose =
                        isRedAlliance
                                ? Constants.Auto.FieldPoses.RED_NODE_SEVEN_GOAL
                                : Constants.Auto.FieldPoses.BLUE_NODE_THREE_GOAL;
                break;
            case EIGHT:
                switch (_goalLevel) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleCubeSetpoint;
                        _endEffector.setState(IntakeState.GROUND);
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleCubePlaceSetpoint;
                        _endEffector.setState(IntakeState.CUBE);
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highCubePlaceSetpoint;
                        _endEffector.setState(IntakeState.CUBE);
                        break;
                }
                pose =
                        isRedAlliance
                                ? Constants.Auto.FieldPoses.RED_NODE_EIGHT_GOAL
                                : Constants.Auto.FieldPoses.BLUE_NODE_TWO_GOAL;
                break;
            case NINE:
                switch (_goalLevel) {
                    case LOW:
                        setpoint = SuperStructureSetpoints.idleConeSetpoint;
                        _endEffector.setState(IntakeState.GROUND);
                        break;
                    case MIDDLE:
                        setpoint = SuperStructureSetpoints.middleConePlaceSetpoint;
                        _endEffector.setState(IntakeState.CONE);
                        break;
                    case HIGH:
                        setpoint = SuperStructureSetpoints.highConePlaceSetpoint;
                        _endEffector.setState(IntakeState.CONE);
                        break;
                }
                pose =
                        isRedAlliance
                                ? Constants.Auto.FieldPoses.RED_NODE_NINE_GOAL
                                : Constants.Auto.FieldPoses.BLUE_NODE_ONE_GOAL;
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
