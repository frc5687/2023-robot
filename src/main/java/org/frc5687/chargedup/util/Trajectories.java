package org.frc5687.chargedup.util;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.Pair;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

/** This class contains all the trajectories we generate from path-planner; */
public class Trajectories {
    private HashMap<String, PathPlannerTrajectory> trajectories = new HashMap<>();
    private static final List<Pair<String, PathConstraints>> paths = new ArrayList<>(Arrays.asList(
            new Pair<>("RED_NODE_ONE_GOAL_ONE", new PathConstraints(3.0, 2.0)),
            new Pair<>("RED_NODE_ONE_GOAL_ONE", new PathConstraints(3.0, 2.0)),
            new Pair<>("RED_NODE_ONE_GOAL_ONE", new PathConstraints(3.0, 2.0))
    ));
    public Trajectories(PathConstraints constraints) {
        for (var pair : paths) {
            trajectories.put(pair.getFirst(), PathPlanner.loadPath(pair.getFirst(), pair.getSecond()));
        }
    }

    public PathPlannerTrajectory getTrajectory(String name) {
        return trajectories.get(name);
    }
}
