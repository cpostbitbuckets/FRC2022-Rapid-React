package frc.robot.utils.modifiedswervelib;

import com.ctre.phoenix.sensors.CANCoder;

public class BitBucketsCanCoderAbsoluteConfiguration {
    private final CANCoder canCoder;
    private final double offset;

    public BitBucketsCanCoderAbsoluteConfiguration(CANCoder canCoder, double offset) {
        this.canCoder = canCoder;
        this.offset = offset;
    }

    public CANCoder getCanCoder() {
        return canCoder;
    }

    public double getOffset() {
        return offset;
    }

}
