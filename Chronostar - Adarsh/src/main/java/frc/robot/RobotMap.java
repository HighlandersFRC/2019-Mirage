package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveForward;
import frc.robot.subsystems.DriveTrain;

public class RobotMap{
    public RobotMap(){

    }

    public static int leftMasterID = 6;
    public static int leftFollower1ID = 5;
    public static int leftFollower2ID = 4;
    public static int rightFollower2ID = 3;
    public static int rightFollower1ID = 2;
    public static int rightMasterID = 1;

    public static TalonSRX leftFollower2 = new TalonSRX(leftFollower2ID);
    //public static TalonFX leftFollower1 = new TalonFX(leftFollower1ID);
    //public static TalonFX rightFollower2 = new TalonFX(rightFollower2ID);
    public static TalonSRX rightFollower1 = new TalonSRX(rightFollower1ID);
    public static TalonSRX leftMaster = new TalonSRX(leftMasterID);
    public static TalonSRX rightMaster = new TalonSRX(rightMasterID);

    public static Relay visionRelay1 = new Relay(0);

    public static AHRS ahrs = new AHRS(SerialPort.Port.kMXP);

    //private static DriveEncoder leftMainDrive = new DriveEncoder(RobotMap.leftDriveLead,RobotMap.leftDriveLead.getSelectedSensorPosition(0));
	//private static DriveEncoder rightMainDrive = new DriveEncoder(RobotMap.rightDriveLead,RobotMap.rightDriveLead.getSelectedSensorPosition(0));

    public static DriveTrain driveTrain = new DriveTrain();


    public static TalonSRX driveMotors[] = {
      RobotMap.leftFollower2,
      RobotMap.rightFollower1,
      RobotMap.leftMaster,
      RobotMap.rightMaster,  
    };

    public static final double kF = 0.153;
    public static final double kP = 0.35025;
    //public static final double kF = 0;
    //public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final int pidSlot = 0;
    private static double targetVelocityTics;

    public static double ticsToInches(double encoderTics) {
      double inches = (encoderTics * 6 * Math.PI)/11264;
      return inches;
    }
  
    public static int inchesToTics(double inches) {
      int encoderTics = (int) Math.round((inches * 11264) / 6);
      encoderTics = (int) Math.round(encoderTics/Math.PI);
      return encoderTics;
    }

    public static double ftPerSecToNIPer100ms(double feetPerSecond) {
      return((((feetPerSecond*12)/10)/(6*Math.PI)) * 11264);
    }

    

    public static void setCameraAPID() {

    }

}