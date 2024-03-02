using System;

public class PIDController
{
    private double Kp;              // Proportional gain
    private double Ki;              // Integral gain
    private double Kd;              // Derivative gain
    private double SetPoint;        // Desired temperature
    private double PrevError;       // Previous error
    private double Integral;        // Integral term
    private double OutputMin;       // Minimum output (e.g., heater power)
    private double OutputMax;       // Maximum output (e.g., heater power)
    private bool ManualTuning;      // Flag for manual tuning mode

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
        ManualTuning = false;
    }

    // Method to compute PID control output
    public double Compute(double currentTemperature)
    {
        double error = SetPoint - currentTemperature;

        // Proportional term
        double proportional = Kp * error;

        // Anti-windup for integral term
        if (!ManualTuning)
        {
            Integral += error;
            Integral = Math.Max(OutputMin / Ki, Math.Min(OutputMax / Ki, Integral));
        }

        // Derivative term
        double derivative = Kd * (error - PrevError);
        PrevError = error;

        // Compute PID output
        double output = proportional + Ki * Integral + derivative;

        // Limit the output within the specified range
        output = Math.Max(OutputMin, Math.Min(OutputMax, output));

        return output;
    }

    // Method to enable manual tuning mode
    public void EnableManualTuning()
    {
        ManualTuning = true;
    }

    // Method to disable manual tuning mode
    public void DisableManualTuning()
    {
        ManualTuning = false;
    }

    // Method to set the setpoint dynamically
    public void SetSetpoint(double setpoint)
    {
        SetPoint = setpoint;
    }
}

public class ThermalSystem
{
    private PIDController controller;

    // Constructor to initialize the thermal system
    public ThermalSystem()
    {
        // Initialize PID controller with initial tuning parameters
        controller = new PIDController(kp: 1.0, ki: 0.1, kd: 0.01, setPoint: 70.0, outputMin: 0.0, outputMax: 100.0);
    }

    // Method to run the control loop of the thermal system
    public void RunControlLoop()
    {
        // Simulate thermal system control loop
        while (true)
        {
            double currentTemperature = ReadTemperature();                  // Read current temperature from sensor
            double heaterPower = controller.Compute(currentTemperature);    // Compute heater power using PID
            SetHeaterPower(heaterPower);                                    // Set heater power
            Console.WriteLine($"Temperature: {currentTemperature} | Heater Power: {heaterPower}");

            // Simulate dynamic setpoint changes
            if (currentTemperature > 80)
            {
                controller.SetSetpoint(60);
            }
            else if (currentTemperature < 65)
            {
                controller.SetSetpoint(75);
            }

            // Sleep for a short duration
            System.Threading.Thread.Sleep(1000);
        }
    }

    // Method to read temperature from the sensor (placeholder implementation)
    private double ReadTemperature()
    {
        // Simulated temperature reading
        Random rand = new Random();
        return 60 + rand.NextDouble() * 40; // Random temperature between 60 and 100
    }

    // Method to set heater power (placeholder implementation)
    private void SetHeaterPower(double power)
    {
        // Placeholder for setting heater power
        // You would implement the actual code to control the heater power here
        Console.WriteLine($"Heater power set to: {power}");
    }
}

public class Program
{
    public static void Main(string[] args)
    {
        ThermalSystem system = new ThermalSystem();
        system.RunControlLoop();
    }
}
