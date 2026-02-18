package frc.robot.Util;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public final class SparkUtil {
    private SparkUtil() {}

    public static boolean sparkStickyFault = false;

    public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
        double value = supplier.getAsDouble();
        if (spark.getLastError() == REVLibError.kOk) {
            consumer.accept(value);
        } else {
            sparkStickyFault = true;
        }
    }

    public static void ifOk(SparkBase spark, DoubleSupplier[] suppliers, Consumer<double[]> consumer) {
        double[] values = new double[suppliers.length];
        for (int index = 0; index < suppliers.length; index++) {
            values[index] = suppliers[index].getAsDouble();
            if (spark.getLastError() != REVLibError.kOk) {
                sparkStickyFault = true;
                return;
            }
        }
        consumer.accept(values);
    }

    public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
        for (int attemptIndex = 0; attemptIndex < maxAttempts; attemptIndex++) {
            REVLibError error = command.get();
            if (error == REVLibError.kOk) {
                return;
            }
            sparkStickyFault = true;
        }
    }
}
