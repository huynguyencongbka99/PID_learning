using System;
using System.IO;

public class PIDController
{
    private double Kp;              // Proportional gain
    private double Ki;              // Integral gain
    private double Kd;              // Derivative gain
    private double SetPoint;        // Desired pressure
    private double PrevError;       // Previous error
    private double Integral;        // Integral term
    private double OutputMin;       // Minimum output (e.g., valve position)
    private double OutputMax;       // Maximum output (e.g., valve position)

    // Constructor to initialize PID controller parameters
    public PIDController(double kp, double ki, double kd, double setPoint, double outputMin, double outputMax)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        SetPoint = setPoint;
        OutputMin = outputMin;
        OutputMax = outputMax;
        PrevError = 0;
        Integral = 0;
    }

    // Method to compute PID control output
    public double Compute(double currentPressure)
    {
        double error = SetPoint - currentPressure;

        // Proportional term
        double proportional = Kp * error;

        // Integral term
        Integral += error;
        double integral = Ki * Integral;

        // Derivative term
        double derivative = Kd * (error - PrevError);
        PrevError = error;

        // Compute PID output
        double output = proportional + integral + derivative;

        // Limit the output within the specified range
        output = Math.Max(OutputMin, Math.Min(OutputMax, output));

        return output;
    }
}

public class HydraulicSystem
{
    private PIDController controller;
    private StreamWriter logFile;
    private Random rand;

    // Constructor to initialize the hydraulic system and open a log file
    public HydraulicSystem(double kp, double ki, double kd, double setPoint, double outputMin, double outputMax)
    {
        // Initialize PID controller with tuning parameters
        controller = new PIDController(kp, ki, kd, setPoint, outputMin, outputMax);

        // Open a log file for recording data
        logFile = new StreamWriter("hydraulic_log.txt");

        rand = new Random();
    }

    // Method to run the control loop of the hydraulic system
    public void RunControlLoop()
    {
        try
        {
            // Simulate hydraulic system control loop
            while (true)
            {
                double currentPressure = ReadPressure();                          // Read current pressure from sensor
                double valvePosition = controller.Compute(currentPressure);      // Compute valve position using PID
                SetValvePosition(valvePosition);                                  // Set valve position
                LogData(currentPressure, valvePosition);                         // Log pressure and valve position
                Console.WriteLine($"Pressure: {currentPressure} | Valve Position: {valvePosition}");

                // Sleep for a short duration
                System.Threading.Thread.Sleep(1000);
            }
        }
        catch (Exception ex)
        {
            Console.WriteLine($"An error occurred: {ex.Message}");
        }
        finally
        {
            logFile.Close();                                                    // Close the log file
        }
    }

    // Method to read pressure from the sensor (placeholder implementation)
    private double ReadPressure()
    {
        // Simulated pressure reading with noise
        double pressureNoise = (rand.NextDouble() - 0.5) * 5.0; // Noise between -2.5 and 2.5
        return 100 + pressureNoise; // Simulated pressure around 100 psi
    }

    // Method to set valve position (placeholder implementation)
    private void SetValvePosition(double position)
    {
        // Placeholder for setting valve position
        // You would implement the actual code to control the valve position here
        Console.WriteLine($"Valve position set to: {position}");
    }

    // Method to log pressure and valve position to a file
    private void LogData(double pressure, double valvePosition)
    {
        string logEntry = $"{DateTime.Now}\t{pressure}\t{valvePosition}";
        logFile.WriteLine(logEntry);
    }
}

public class Program
{
    public static void Main(string[] args)
    {
        // PID controller parameters
        double kp = 1.0;
        double ki = 0.1;
        double kd = 0.01;
        double setPoint = 100.0;
        double outputMin = 0.0;
        double outputMax = 100.0;

        HydraulicSystem system = new HydraulicSystem(kp, ki, kd, setPoint, outputMin, outputMax);
        system.RunControlLoop();
    }
}
