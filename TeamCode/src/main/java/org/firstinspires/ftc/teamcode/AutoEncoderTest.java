package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Date;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcValueMenu;
import trclib.TrcRobot;

/**
 * Created by pmartin on 11/18/2017.
 */

@Autonomous(name="EncoderTest", group="TRCLib")
/* HOW TO USE THIS OPMODE:

  1. Position robot at a marked starting position
  2. On the menu, specify the drive axis (X = sideways, Y = forward)
  3. Run the op mode 10 times per axis, recording these values:
        LF (left front) from telemetry
        LR (left rear) from telemetry
        RF (right front) from telemetry
        RR (right rear) from telemetry
        Distance travelled in inches by manual measurement
  5. After all the runs are complete, compute the average value per wheel per axis. For example:
        LF(X) = average of LF values for X axis
        LF(Y) = average of LF values for Y axis
  6. Compute average distance travelled per axis
        D(X) = average of X distances
        D(Y) = average of Y distances
  7. Compute overall average encoder value per axis:
        E(X) = ((LF(X) + RR(X)) - (RF(X) + LR(X))) / 4.0;
        E(Y) = (LF(Y) + LR(Y) + RF(Y) + RR(Y)) / 4.0;
  8. Compute inches per tick per axis
        I(X) = D(X) / E(X)
        I(Y) = D(Y) / E(Y)
  9. Use this value for the constants in RobotInfo.java
        ENCODER_X_INCHES_PER_COUNT = I(X)
        ENCODER_Y_INCHES_PER_COUNT = I(Y)
*/

public class AutoEncoderTest extends FtcOpMode {
    private static final String moduleName = "FtcAuto";
    enum Axis {
        X,
        Y
    }

    private Robot robot;
    private TrcRobot.RobotCommand autoCommand = null;
    private double driveTime = 0.8;
    private double drivePower = 0.2;
    private Axis driveAxis = Axis.Y;

    @Override
    public void initRobot() {
        robot = new Robot(TrcRobot.RunMode.AUTO_MODE);

        doMenus();

        double xpower = 0.0;
        double ypower = 0.0;

        if (driveAxis == Axis.X) {
            xpower = drivePower;
            ypower = 0;
        } else {
            xpower = 0;
            ypower = drivePower;
        }

        autoCommand = new CmdTimedDrive(robot, 0.0, driveTime, xpower, ypower, 0.0);
    }

    @Override
    public void startMode() {
        Date now = new Date();

        robot.startMode(TrcRobot.RunMode.AUTO_MODE);

        //Explicitly reset hardware motor encoders so we can read values when finished
        robot.driveBase.resetPosition(true);

        robot.battery.setEnabled(true);
        robot.tracer.traceInfo(moduleName, "%s: ***** Starting autonomous *****", now.toString());
        robot.dashboard.clearDisplay();
    }

    @Override
    public void stopMode() {
        robot.stopMode(TrcRobot.RunMode.AUTO_MODE);
        //Display encoder info for all motors
        robot.dashboard.displayPrintf(1, "Encoder: LF=%.1f, LR=%.1f, RF=%.1f, RR=%.1f",
                robot.leftFrontWheel.getPosition(), robot.leftRearWheel.getPosition(), robot.rightFrontWheel.getPosition(), robot.rightRearWheel.getPosition());


        robot.battery.setEnabled(false);
    }

    @Override
    public void runContinuous(double elapsedTime) {
        if (autoCommand != null)
        {
            autoCommand.cmdPeriodic(elapsedTime);
        }
    }

    private void doMenus() {
        FtcChoiceMenu<Axis> driveAxisMenu = new FtcChoiceMenu<>("Drive Axis:", null, robot);

        FtcMenu.walkMenuTree(driveAxisMenu, this);

        driveAxis = driveAxisMenu.getCurrentChoiceObject();

        robot.dashboard.displayPrintf(1, "Drive: Axis=%s, Time=%.0f, Power=%.1f",
                driveAxis.toString(),driveTime, drivePower);
    }
}
