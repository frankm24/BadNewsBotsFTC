package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;

@I2cSensor(name = "Nvidia AI Computer", description = "Computer to run Neural Networks and Computer Vision code.", xmlTag = "AIComp")
public class NvidiaComputer extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x18);

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }
    @Override
    protected synchronized boolean doInitialize() {
        return true;
    }
    @Override
    public String getDeviceName() {
        return "Nvidia AI Computer";
    }
    public enum Register {
        FIRST(0),
        TEST(0x01),
        TEST2(0x02),
        LAST(TEST.bVal);

        public int bVal;
        Register(int bVal) {
            this.bVal = bVal;
        }
    }
    protected void setOptimalReadWindow() {
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.LAST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }
    public NvidiaComputer(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }
}
