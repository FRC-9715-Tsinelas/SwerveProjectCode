package frc.robot;
import frc.robot.Constants.botaimConstants;

public void MagicFunction(double distance) {
  power = botaimConstants.kMagicFunctionA * Math.pow(botaimConstants.kMagicFunctionB, distance)
  return power
}
