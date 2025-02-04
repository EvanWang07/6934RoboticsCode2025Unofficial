package frc.robot;

public final class BasicOperations {
    public static double getSuccessRate(boolean[] attemptsList) {
        int attempts = 0;
        int successes = 0;
        for (boolean attemptElement : attemptsList) {
            attempts++;
            if (attemptElement) {
                successes++;
            }
        }
        double successRate = (double) successes / attempts;
        return successRate;
    }

    public static void insertBooleanToConfinedList(boolean[] list, boolean newValue) {
        for (int i = list.length - 1; i > 0; i--) {
            list[i] = list[i - 1];
        }
        list[0] = newValue;
    }

    public static void insertDoubleToConfinedList(double[] list, double newValue) {
        for (int i = list.length - 1; i > 0; i--) {
            list[i] = list[i - 1];
        }
        list[0] = newValue;
    }

    public static double findAverage(double numOne, double numTwo) {
        return (numOne + numTwo) / 2;
    }
}
