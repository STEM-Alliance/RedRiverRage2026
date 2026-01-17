package frc.robot.subsystems;
import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

  /**
   * This class reads the CAN bus and returns the detected distance of the sensor.
   * The wpilib CAN bus is defined here: https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html
   * At a high level the CAN ID's are broken down into the following:
   * 5 Bits: Device Type (Defined as 10 for Miscellaneous device)
   * 8 BIts: Manufacturer Code (defined as 20 values 17-255 are reserved)
   * 10 Bits: API (See below)
   * 4 BIts: Device Number (Configured by user NOTE: NOT IMPLEMENTED)
   * 
   * Note the API is split up into API Class (6 bits) and API Index (4 bits).
   * The Time-Of-Flight (TOF) Distance Sensor device uses the API Class of 5 (Status) and API Index of 0 for distance
   * 
   * Here is the layout of the CAN ID from the device itself
   * #define WPILIB_DEVICE_TYPE 	10 << 24
   * #define WPILIB_MFG_CODE 	20 << 16 
   * #define WPILIB_API_CLASS	5 << 10 
   * #define WPILIB_API_INDEX    0 << 6 
   * #define WPILIB_DEV_NUM		0 
   * #define WPILIB_CAN_ID WPILIB_DEVICE_TYPE | WPILIB_MFG_CODE | WPILIB_API_CLASS | WPILIB_API_INDEX | WPILIB_DEV_NUM 
   * 
   * The TOF Distance Sensor device only transmits a single CAN frame. The CAN frame is defined below:
   * Byte 0: Status
   * Byte 1: Distance (MSB)
   * Byte 2: Distance (LSB)
   * Byte 3: Ambient photon count (MSB)
   * Byte 4: Ambient photon count (LSB)
   * Byte 5: Signal photon count (MSB)
   * Byte 6: Signal photon count (LSB)
   * 
   */
public class TofDistanceSubsystem extends SubsystemBase {
    private final int DEVICE_TYPE = 10;
    private final int MFG_CODE = 20;
    private final int API_CLASS = 5;
    private final int API_INDEX = 0; 
    private final int m_timeout = 50;
    private final CAN m_can;

    private CANData m_data = new CANData();
    private int m_lastStatus = -1;
    private int m_lastDistance = -1;
    private int m_lastAmbient = -1;
    private int m_lastSignal = -1;
    private long m_lastTimestamp = RobotController.getFPGATime();

    /*
     * To make this clear refer to wpilib TBD. The APP ID consists of the APP class and the APP Index. The App class 
     */
    int m_appID = ((API_CLASS << 10) | (API_INDEX << 6)) >> 6;

    /**
     * Constructor for the subsystem.
     * @param canID The CAN ID of the device.
     */
    public TofDistanceSubsystem(int canID) {
      m_can = new CAN(canID, MFG_CODE, DEVICE_TYPE);  
    }

    /**
     * This method is called periodically and reads data from the CAN bus.
     * It updates the SmartDashboard with the distance and status.
     */
    public void periodic() {
      if (m_can.readPacketTimeout(m_appID, m_timeout, m_data))
      {
        m_lastStatus = m_data.data[0] & 0xFF;
        m_lastDistance = ((m_data.data[1] & 0xFF) << 8) | (m_data.data[2] & 0xFF);
        m_lastAmbient =  ((m_data.data[3] & 0xFF) << 8) | (m_data.data[4] & 0xFF);
        m_lastSignal =   ((m_data.data[5] & 0xFF) << 8) | (m_data.data[6] & 0xFF);
        m_lastTimestamp = RobotController.getFPGATime();
        //System.out.println("TOF: " + m_data.data[0] + "," + m_data.data[1] + "," + m_data.data[2] + ",");
      }
      else
      {
        m_lastStatus = -1;
        m_lastDistance = -1;
        m_lastAmbient = -1;
        m_lastSignal = -1;
        m_lastTimestamp = RobotController.getFPGATime();
      }

      SmartDashboard.putNumber("TOFDistance", m_lastDistance);
      SmartDashboard.putNumber("TOFStatus", m_lastStatus);
      SmartDashboard.putNumber("TOFAmbient", m_lastAmbient);
      SmartDashboard.putNumber("TOFSignal", m_lastSignal);
  }

    /**
     * Returns the last detected status code.
     * @return The last detected status code.
     */
    public int get_status() {
      return m_lastStatus;
    }

    /**
     * Returns the last detected distance.
     * @return The last detected distance in mm.
     */
    public int get_distance() {
      return m_lastDistance;
    }

    /**
     * Returns the last ambient photon count.
     * @return The last ambient photon count.
     */
    public int get_ambient() {
      return m_lastAmbient;
    }

        /**
     * Returns the last signal photon count.
     * @return The last signal photon count.
     */
    public int get_signal() {
      return m_lastSignal;
    }

    public boolean is_timed_out() {
      return (RobotController.getFPGATime() - m_lastTimestamp) > (m_timeout * 1000);
    }

    /**
     * This should usually be used with a debouncer, 0.06s (3 loops) seems reasonable.
    */
    public boolean is_within_threshold(int threshold) {
      return ((m_lastStatus == 0) && (m_lastDistance < threshold) && (m_lastDistance != -1));
    }
}
