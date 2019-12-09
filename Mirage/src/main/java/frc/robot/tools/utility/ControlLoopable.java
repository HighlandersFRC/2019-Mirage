package frc.robot.tools.utility;

public interface ControlLoopable 
{
	public void controlLoopUpdate();
	public void setPeriodMs(long periodMs);
}