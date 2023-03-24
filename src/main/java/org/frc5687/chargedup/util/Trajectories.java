package org.frc5687.chargedup.util;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import java.util.HashMap;

/** This class contains all the trajectories we generate from path-planner; */
public class Trajectories {
    HashMap<String, PathPlannerTrajectory> trajectories = new HashMap<>();
    String path_names[] = new String[] {
            "RED_NODE_ONE_GOAL_ONE", // to and from top and bottom two nodes and top and bottom two goals
            "RED_NODE_ONE_GOAL_TWO",
            "RED_NODE_TWO_GOAL_ONE",
            "RED_NODE_TWO_GOAL_TWO",
            "RED_NODE_EIGHT_GOAL_THREE",
            "RED_NODE_EIGHT_GOAL_FOUR",
            "RED_NODE_NINE_GOAL_THREE",
            "RED_NODE_NINE_GOAL_FOUR",
            "RED_GOAL_ONE_NODE_ONE",
            "RED_GOAL_ONE_NODE_TWO",
            "RED_GOAL_TWO_NODE_ONE",
            "RED_GOAL_TWO_NODE_TWO",
            "RED_GOAL_THREE_NODE_EIGHT",
            "RED_GOAL_THREE_NODE_NINE",
            "RED_GOAL_FOUR_NODE_EIGHT",
            "RED_GOAL_FOUR_NODE_NINE",

            "RED_BUMP_GOAL_ONE",
            "RED_GOAL_ONE_CHARGE_TWO", //specifically for steal cubes auto
            "RED_CHARGE_TWO_GOAL_TWO",
            "RED_GOAL_TWO_CHARGE_THREE",
            "RED_CHARGE_THREE_GOAL_THREE",
            "RED_GOAL_THREE_CHARGE_FOUR",
            "RED_NOBUMP_GOAL_FOUR",

            "BLUE_NODE_ONE_GOAL_ONE", // to and from top and bottom two nodes and top and bottom two goals
            "BLUE_NODE_ONE_GOAL_TWO",
            "BLUE_NODE_TWO_GOAL_ONE",
            "BLUE_NODE_TWO_GOAL_TWO",
            "BLUE_NODE_EIGHT_GOAL_THREE",
            "BLUE_NODE_EIGHT_GOAL_FOUR",
            "BLUE_NODE_NINE_GOAL_THREE",
            "BLUE_NODE_NINE_GOAL_FOUR",
            "BLUE_GOAL_ONE_NODE_ONE",
            "BLUE_GOAL_ONE_NODE_TWO",
            "BLUE_GOAL_TWO_NODE_ONE",
            "BLUE_GOAL_TWO_NODE_TWO",
            "BLUE_GOAL_THREE_NODE_EIGHT",
            "BLUE_GOAL_THREE_NODE_NINE",
            "BLUE_GOAL_FOUR_NODE_EIGHT",
            "BLUE_GOAL_FOUR_NODE_NINE",

            "BLUE_BUMP_GOAL_ONE",
            "BLUE_GOAL_ONE_CHARGE_TWO", //specifically for steal cubes auto
            "BLUE_CHARGE_TWO_GOAL_TWO",
            "BLUE_GOAL_TWO_CHARGE_THREE",
            "BLUE_CHARGE_THREE_GOAL_THREE",
            "BLUE_GOAL_THREE_CHARGE_FOUR"
    };
    public Trajectories(PathConstraints constraints) {
        for (var name : path_names) {
            trajectories.put(name, PathPlanner.loadPath(name, constraints));
        }
    }

    public PathPlannerTrajectory getTrajectory(String name) {
        return trajectories.get(name);
    }
}
