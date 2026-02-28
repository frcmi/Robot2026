package frc.robot.constants;

import frc.robot.lib.LoggedTunableNumber;

public class DriveConstants {

  // p gain for the drive x and y controllers (side to side and forward/back)
  public static LoggedTunableNumber TRANSLATION_KP =
      new LoggedTunableNumber("Drive/TranslationKp", 1.0);
  public static LoggedTunableNumber TRANSLATION_KD =
      new LoggedTunableNumber("Drive/TranslationKd", 0.0);
  public static LoggedTunableNumber PP_ANGLE_KP = new LoggedTunableNumber("Drive/PPAngleKp", 5.0);
  public static LoggedTunableNumber PP_ANGLE_KD = new LoggedTunableNumber("Drive/PPAngleKd", 0.4);

  public static final double DEADBAND = 0.1;

  // p gain for the drive angle controller
  public static final double ANGLE_KP = 5.0;
  public static final double ANGLE_KD = 0.4;
  public static final double ANGLE_MAX_VELOCITY = 8.0;
  public static final double ANGLE_MAX_ACCELERATION = 20.0;
  public static final double FF_START_DELAY = 2.0; // Secs
  public static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  public static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  public static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  public static final double RIGHT_TRENCH_Y = 0.625;
  public static final double LEFT_TRENCH_Y = 7.425;

  public static final double MAX_SPEED_MULTIPLIER = 0.75;
  public static final double MAX_ROTATION_MULTIPLIER = 0.75;

  public static final double INTAKING_DRIVETRAIN_SPEED_MULTIPLIER = 0.5;
  public static final double INTAKING_DRIVETRAIN_ROTATION_MULTIPLIER = 1.0;
  public static final double TRANSFERRING_DRIVETRAIN_SPEED_MULTIPLIER = 0.5;
  public static final double TRANSFERRING_DRIVETRAIN_ROTATION_MULTIPLIER = 1.0;
}
