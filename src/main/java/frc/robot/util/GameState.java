package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

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
        AUTO,
        TRANSITION,
        SHIFT_1,
        SHIFT_2,
        SHIFT_3,
        SHIFT_4,
        ENDGAME,
        UNDEFINED;
    }

    public static Period getPeriod(double shooterTimeDelay) {
        if (DriverStation.isAutonomous()) {
            return Period.AUTO;
        }
        double timeRemaining;
        timeRemaining = DriverStation.getMatchTime();
        if (timeRemaining < 30 - shooterTimeDelay) {
            return Period.ENDGAME;
        }
        if (timeRemaining < 55 - shooterTimeDelay) {
            return Period.SHIFT_4;
        }
        if (timeRemaining < 80 - shooterTimeDelay) {
            return Period.SHIFT_3;
        }
        if (timeRemaining < 105 - shooterTimeDelay) {
            return Period.SHIFT_2;
        }
        if (timeRemaining < 130 - shooterTimeDelay) {
            return Period.SHIFT_1;
        }
        if (timeRemaining < 140 - shooterTimeDelay) {
            return Period.TRANSITION;
        } else {
            return Period.UNDEFINED;
        }
    }

    public static boolean isHubActive(double shooterTimeDelay) {
        Period period = getPeriod(shooterTimeDelay);
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
