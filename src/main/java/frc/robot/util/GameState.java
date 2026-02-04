package frc.robot.util;

import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class GameState {
    public static Alliance getAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return Alliance.Red;
        }
        return alliance.get();
    }

    public static Alliance getautonwinnerAlliance() {
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.equals("R")) {
            return Alliance.Red;
        } else {
            return Alliance.Blue;
        }
    }

    public static boolean areWeFirst() {
        Alliance alliance = getAlliance();
        Alliance winnerAlliance = getautonwinnerAlliance();
        if (alliance == Alliance.Blue && Alliance.Red == winnerAlliance) {
            return true;
        } else if (alliance == Alliance.Red && Alliance.Blue == winnerAlliance) {
            return true;
        } else {
            return false;
        }
    }

    public enum Period {
        AUTO, TRANSITION, SHIFT_1, SHIFT_2, SHIFT_3, SHIFT_4, ENDGAME, UNDEFINED;
    }

    public static Period getPeriod() {
        if (DriverStation.isAutonomous()) {
            return Period.AUTO;
        }
        double timeRemaining;
        timeRemaining = DriverStation.getMatchTime();
        if (timeRemaining < 30) {
            return Period.ENDGAME;
        }
        if (timeRemaining < 55) {
            return Period.SHIFT_4;
        }
        if (timeRemaining < 80) {
            return Period.SHIFT_3;
        }
        if (timeRemaining < 105) {
            return Period.SHIFT_2;
        }
        if (timeRemaining < 130) {
            return Period.SHIFT_1;
        }
        if (timeRemaining < 140) {
            return Period.TRANSITION;
        } else {
            return Period.UNDEFINED;
        }
    }

    public static boolean isHubActive() {
        Period period = getPeriod();
        if (period == Period.AUTO || period == Period.TRANSITION || period == Period.ENDGAME) {
            return true;
        } else if (period == Period.SHIFT_1 || period == Period.SHIFT_3) {
            return areWeFirst();
        } else if (period == Period.SHIFT_2 || period == Period.SHIFT_4) {
            return !areWeFirst();
        } else {
            return false;
        }
    }
}
