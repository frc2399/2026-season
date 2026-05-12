// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.FieldConstants;
import frc.robot.util.GameState;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.BiConsumer;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer robotContainer;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
        DriverStation.silenceJoystickConnectionWarning(true);
        FollowPathCommand.warmupCommand();
        PathfindingCommand.warmupCommand();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        robotContainer.visionPoseEstimator.periodic();
        robotContainer.setAlerts();
        SmartDashboard.putNumber("robot/batteryVoltage", RobotController.getBatteryVoltage());
        SmartDashboard.putBoolean("robot/0 second delayIsHubActive", GameState.isHubActive(0));
        SmartDashboard.putBoolean("robot/1 second delayIsHubActive", GameState.isHubActive(1));
        SmartDashboard.putNumber("robot/match time", DriverStation.getMatchTime());
        SmartDashboard.putString("robot/alliance", FieldConstants.alliance.toString());
        SmartDashboard.putBoolean(
                "robot/has stopped shooting",
                robotContainer.autonCommandFactory.hasStoppedShooting());
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        robotContainer.disableSubsystems();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void robotInit() {
        CommandScheduler.getInstance()
                .onCommandInitialize(cmd -> DataLogManager.log(cmd.getName() + " : Init"));
        CommandScheduler.getInstance()
                .onCommandInterrupt(
                        (interrupted, interrupting) ->
                                DataLogManager.log(
                                        interrupted.getName()
                                                + "Interrupted by "
                                                + (!interrupting.isEmpty()
                                                        ? interrupting.get().getName()
                                                        : "nothing")));
        CommandScheduler.getInstance()
                .onCommandFinish(cmd -> DataLogManager.log(cmd.getName() + ": End"));
        SmartDashboard.putString(
                "branch and date", BuildConstants.GIT_BRANCH + " " + BuildConstants.GIT_DATE);
        Map<String, Integer> commandCounts = new HashMap<>();
        BiConsumer<Command, Boolean> logCommandFunction =
                (Command command, Boolean active) -> {
                    String name = command.getName();
                    int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
                    commandCounts.put(name, count);
                    SmartDashboard.putBoolean(
                            "CommandsUnique/"
                                    + name
                                    + "_"
                                    + Integer.toHexString(command.hashCode()),
                            active);
                    SmartDashboard.putBoolean("CommandsAll/" + name, count > 0);
                };
        CommandScheduler.getInstance()
                .onCommandInitialize(
                        (Command command) -> {
                            logCommandFunction.accept(command, true);
                            DataLogManager.log(command.getName() + " : Init");
                        });
        CommandScheduler.getInstance()
                .onCommandFinish(
                        (Command command) -> {
                            logCommandFunction.accept(command, false);
                            DataLogManager.log(command.getName() + ": End");
                        });
        CommandScheduler.getInstance()
                .onCommandInterrupt(
                        (interrupted, interrupting) -> {
                            logCommandFunction.accept(interrupted, false);
                            DataLogManager.log(
                                    interrupted.getName()
                                            + " Interrupted by "
                                            + (!interrupting.isEmpty()
                                                    ? interrupting.get().getName()
                                                    : "nothing"));
                        });
    }

    @Override
    public void driverStationConnected() {
        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red) {
            FieldConstants.alliance = Optional.of(Alliance.Red);
        } else {
            FieldConstants.alliance = Optional.of(Alliance.Blue);
        }
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        robotContainer.updateSimulation();
    }
}
