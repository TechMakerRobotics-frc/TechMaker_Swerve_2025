package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.IOException;
import org.json.simple.parser.ParseException;

/** Command to execute a Choreo trajectory using PathPlanner. */
public class ChoreoAuto extends Command {

    private final String traj;
    private final double timeOut;
    private final Timer time = new Timer();
    private Command pathfollower;

    /**
     * Constructs a ChoreoAuto command.
     *
     * @param traj The name of the Choreo trajectory to load.
     * @param timeOut Timeout for the command execution.
     */
    public ChoreoAuto(String traj, double timeOut) {
        this.traj = traj;
        this.timeOut = timeOut;
    }

    @Override
    public void initialize() {
        time.reset();
        time.start();
        try {
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(traj);
            pathfollower = AutoBuilder.followPath(path);
            pathfollower.initialize();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (ParseException e) {
            e.printStackTrace();
        } catch (FileVersionException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void execute() {
        if (pathfollower != null) {
            pathfollower.execute();
        }
    }

    @Override
    public boolean isFinished() {
        return pathfollower == null || pathfollower.isFinished() || time.hasElapsed(timeOut);
    }

    @Override
    public void end(boolean interrupted) {
        time.stop();
        if (pathfollower != null) {
            pathfollower.end(interrupted);
        }
    }
}
