package frc.robot.util;

import java.util.HashMap;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;

public final class DataLogHelpers {
    private static final String m_dataLogHeader = "7048/";
    private static final DataLog m_dataLog = DataLogManager.getLog();

    // Other LogEntry types can be added, but for now only booleans and doubles seem useful.
    private static final HashMap<String, BooleanLogEntry> m_booleanLogEntries = new HashMap<>();
    private static final HashMap<String, DoubleLogEntry> m_doubleLogEntries = new HashMap<>();

    /**
     * Logs a boolean to the {@code DataLog} and optionally sends it to the dashboard.
     * 
     * @param value The value of the boolean to be logged.
     * @param name The name of the booleans log entry (appended to the m_dataLogHeader).
     * @param sendToDashboard Whether or not the boolean should also be sent to the dashboard. <b>
     * The sent dashboard value will also be logged under the SmartDashboard log</b>, unless
     * {@code DataLogManager.logNetworkTables(false)} is called.
    */
    public static final void logBoolean(boolean value, String name, boolean... sendToDashboard) {
        // If a log entry does not exist under the name, a new log entry is created. The
        // value is always appended to the log entry, whether it was newly created or not.
        m_booleanLogEntries.computeIfAbsent(
            name,
            __ -> new BooleanLogEntry(m_dataLog, m_dataLogHeader + name)
        ).append(value);

        // sendToDashboard uses varargs instead of function overloading so that it can be excluded.
        // This also means that any number of booleans can be after the name, but only [0] is used.
        if (sendToDashboard.length > 0 && sendToDashboard[0]) {
            SmartDashboard.putBoolean(name, value);
        }
    }

    /**
     * Logs a double to the {@code DataLog} and optionally sends it to the dashboard.
     * 
     * @param value The value of the double to be logged.
     * @param name The name of the doubles log entry (appended to the m_dataLogHeader).
     * @param sendToDashboard Whether or not the double should also be sent to the dashboard. <b>
     * The sent dashboard value will also be logged under the SmartDashboard log</b>, unless
     * {@code DataLogManager.logNetworkTables(false)} is called.
    */
    public static final void logDouble(double value, String name, boolean... sendToDashboard) {
        // If a log entry does not exist under the name, a new log entry is created. The
        // value is always appended to the log entry, whether it was newly created or not.
        m_doubleLogEntries.computeIfAbsent(
            name,
            __ -> new DoubleLogEntry(m_dataLog, m_dataLogHeader + name)
        ).append(value);

        // sendToDashboard uses varargs instead of function overloading so that it can be excluded.
        // This also means that any number of booleans can be after the name, but only [0] is used.
        if (sendToDashboard.length > 0 && sendToDashboard[0]) {
            SmartDashboard.putNumber(name, value);
        }
    }
}
