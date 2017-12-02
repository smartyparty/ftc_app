package org.firstinspires.ftc.teamcode;

import trclib.TrcUtil;

enum Alliance {
    RED,
    BLUE
}

enum StartPos
{
    NEAR,
    FAR
}

/**
 * Created by pmartin on 12/1/2017.
 */

public class SharedFunctions {
    public static void displaceJewel(Robot robot, Robot.ObjectColor jewelColor, Alliance alliance, double power, double driveTime) {
        //Absolute drive distance required to displace a jewel
        //final double displacement_distance = 6;
        double xDrivePower = 0.0;
        double yDrivePower = 0.0;
        double turnPower = 0.0;
//        double targetX = 0.0;
//        double targetY = 0.0;

        //Drive at half-power
        //robot.encoderYPidCtrl.setOutputRange(-0.5, 0.5);
        //No movement along x-axis
        //targetX = 0.0;
        //No gyro heading
        //robot.targetHeading = 0.0;

        //ASSUMPTION: jewel color sensor faces forward
        //Determine which direction to move to displace the correct jewel by specifying
        //a Y-axis drive target value of  + or - the desired distance
        if (jewelColor == Robot.ObjectColor.RED && alliance == Alliance.RED ||
                jewelColor == Robot.ObjectColor.BLUE && alliance == Alliance.BLUE) {
            //drive backward
            //targetY = -displacement_distance;

            yDrivePower = power;
        } else if (jewelColor == Robot.ObjectColor.BLUE && alliance == Alliance.RED ||
                jewelColor == Robot.ObjectColor.RED && alliance == Alliance.BLUE) {
            //drive forward
            //targetY = displacement_distance;
            yDrivePower = -power;
        }

        driveForTime(robot, xDrivePower, yDrivePower, turnPower, driveTime);
    }

    public static void driveToSafeZone(Robot robot, Alliance alliance, StartPos startPosition, double power) {
        double xpower = 0.0;
        double ypower = 0.0;
        double driveTime = 0.0;
        double xSign = (alliance == Alliance.RED ? 1.0 : -1.0);
        double ySign = (alliance == Alliance.RED ? 1.0 : -1.0);

        //Blue advances to safezone forward (toward from color sensor / negative sign)
        //Red advances to safezone rearward (away from color sensor / positive sign)

        //Blue arena center = -x
        //Red arena center = +x

        switch (startPosition) {
            case NEAR:
                //drive forward
                ypower = power * ySign;
                driveTime = 1;
                driveForTime(robot, xpower,ypower,0, driveTime);
                break;
            case FAR:
                //drive sideways
                ypower = 0;
                xpower = power * xSign;
                driveTime = 0.4;
                driveForTime(robot, xpower, ypower, 0, driveTime);

                //drive forward
                driveTime = 1.0;
                xpower = 0;
                ypower = power * ySign;
                driveForTime(robot, xpower, ypower, 0, driveTime);
                break;
        }
    }

    public static void driveForTime(Robot robot, double xpower, double ypower, double turnpower, double time) {
        long start = TrcUtil.getCurrentTimeMillis();
        long expired = 0;
        while (expired < (time * 1000)) {
            robot.driveBase.mecanumDrive_Cartesian(xpower, ypower, turnpower);
            expired = (TrcUtil.getCurrentTimeMillis() - start);
        }

        robot.driveBase.stop();
    }
}
