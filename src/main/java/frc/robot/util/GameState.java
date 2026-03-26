package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;

public class GameState {
    public static Alliance getAlliance() {
        if (FieldConstants.alliance.isEmpty()) {
            return Alliance.Red;
        }
        return FieldConstants.alliance.get();
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
        int ENDGAME_SECONDS_BOUNDARY = 30;
        int SHIFT_FOUR_SECONDS_BOUNDARY = 55;
        int SHIFT_THREE_SECONDS_BOUNDARY = 80;
        int SHIFT_TWO_SECONDS_BOUNDARY = 105;
        int SHIFT_ONE_SECONDS_BOUNDARY = 130;
        int TRANSITION_SECONDS_BOUNDARY = 140;
        if (areWeFirst()) {
            SHIFT_TWO_SECONDS_BOUNDARY += (3 - shooterTimeDelay);
            SHIFT_THREE_SECONDS_BOUNDARY += shooterTimeDelay;
            SHIFT_FOUR_SECONDS_BOUNDARY += (3 - shooterTimeDelay);
        } else {
            SHIFT_ONE_SECONDS_BOUNDARY += (3 - shooterTimeDelay);
            SHIFT_TWO_SECONDS_BOUNDARY += shooterTimeDelay;
            SHIFT_THREE_SECONDS_BOUNDARY += (3 - shooterTimeDelay);
            SHIFT_FOUR_SECONDS_BOUNDARY += shooterTimeDelay;
        }
        if (timeRemaining < ENDGAME_SECONDS_BOUNDARY) {
            return Period.ENDGAME;
        }
        if (timeRemaining < SHIFT_FOUR_SECONDS_BOUNDARY) {
            return Period.SHIFT_4;
        }
        if (timeRemaining < SHIFT_THREE_SECONDS_BOUNDARY) {
            return Period.SHIFT_3;
        }
        if (timeRemaining < SHIFT_TWO_SECONDS_BOUNDARY) {
            return Period.SHIFT_2;
        }
        if (timeRemaining < SHIFT_ONE_SECONDS_BOUNDARY) {
            return Period.SHIFT_1;
        }
        if (timeRemaining < TRANSITION_SECONDS_BOUNDARY) {
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
