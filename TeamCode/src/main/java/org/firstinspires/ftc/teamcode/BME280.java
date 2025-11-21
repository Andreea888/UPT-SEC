package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(name = "BME280", xmlTag = "BME280")
public class BME280 extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    public static final I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x76);


    // --- Register map ---
    public enum Register {
        FIRST(0xD0),
        CHIP_ID(0xD0),
        RESET(0xE0),
        CTRL_HUM(0xF2),
        STATUS(0xF3),
        CTRL_MEAS(0xF4),
        CONFIG(0xF5),
        PRESS_MSB(0xF7),
        PRESS_LSB(0xF8),
        PRESS_XLSB(0xF9),
        TEMP_MSB(0xFA),
        TEMP_LSB(0xFB),
        TEMP_XLSB(0xFC),
        HUM_MSB(0xFD),
        HUM_LSB(0xFE),
        CALIB00(0x88), // calibration start
        CALIB25(0xA1),
        CALIB26(0xE1),
        CALIB41(0xF0),
        LAST(0xFE);

        public final int bVal;
        Register(int bVal) { this.bVal = bVal; }
    }

    // Calibration variables
    private int dig_T1, dig_T2, dig_T3;
    private int dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    private int dig_H1, dig_H2, dig_H3, dig_H4, dig_H5, dig_H6;
    private int t_fine;

    // --- Constructor ---
    public BME280(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        this.setOptimalReadWindow();
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    // --- Read window ---
    protected void setOptimalReadWindow() {
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.PRESS_MSB.bVal,
                Register.LAST.bVal - Register.PRESS_MSB.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    // --- Initialization ---
    @Override
    protected boolean doInitialize() {
        // Reset
        this.deviceClient.write8(Register.RESET.bVal, 0xB6);
        try { Thread.sleep(100); } catch (InterruptedException e) {}

        // Set oversampling
        this.deviceClient.write8(Register.CTRL_HUM.bVal, 0x01); // humidity x1
        this.deviceClient.write8(Register.CTRL_MEAS.bVal, 0x27); // temp/press x1, normal mode
        this.deviceClient.write8(Register.CONFIG.bVal, 0xA0);    // standby 1000ms

        // Read calibration data
        readCalibrationData();

        return true;
    }

    // --- Read calibration data from sensor ---
    private void readCalibrationData() {
        byte[] calib1 = this.deviceClient.read(Register.CALIB00.bVal, 26); // 0x88-0xA1
        byte[] calib2 = this.deviceClient.read(Register.CALIB26.bVal, 7);  // 0xE1-0xE7

        dig_T1 = (calib1[1] << 8 & 0xFF00) | (calib1[0] & 0xFF);
        dig_T2 = (short)((calib1[3] << 8) | (calib1[2] & 0xFF));
        dig_T3 = (short)((calib1[5] << 8) | (calib1[4] & 0xFF));

        dig_P1 = (calib1[7] << 8 & 0xFF00) | (calib1[6] & 0xFF);
        dig_P2 = (short)((calib1[9] << 8) | (calib1[8] & 0xFF));
        dig_P3 = (short)((calib1[11] << 8) | (calib1[10] & 0xFF));
        dig_P4 = (short)((calib1[13] << 8) | (calib1[12] & 0xFF));
        dig_P5 = (short)((calib1[15] << 8) | (calib1[14] & 0xFF));
        dig_P6 = (short)((calib1[17] << 8) | (calib1[16] & 0xFF));
        dig_P7 = (short)((calib1[19] << 8) | (calib1[18] & 0xFF));
        dig_P8 = (short)((calib1[21] << 8) | (calib1[20] & 0xFF));
        dig_P9 = (short)((calib1[23] << 8) | (calib1[22] & 0xFF));

        dig_H1 = calib1[25] & 0xFF;
        dig_H2 = (short)((calib2[1] << 8) | (calib2[0] & 0xFF));
        dig_H3 = calib2[2] & 0xFF;
        dig_H4 = (calib2[3] << 4) | (calib2[4] & 0x0F);
        dig_H5 = ((calib2[4] & 0xF0) >> 4) | (calib2[5] << 4);
        dig_H6 = calib2[6];
    }

    // --- Read temperature (°C) ---
    public double readTemperatureC() {
        byte[] data = this.deviceClient.read(Register.TEMP_MSB.bVal, 3);
        int raw = ((data[0] & 0xFF) << 12) | ((data[1] & 0xFF) << 4) | ((data[2] & 0xF0) >> 4);
        return compensateTemperature(raw);
    }

    // --- Read pressure (hPa) ---
    public double readPressureHPa() {
        byte[] data = this.deviceClient.read(Register.PRESS_MSB.bVal, 3);
        int raw = ((data[0] & 0xFF) << 12) | ((data[1] & 0xFF) << 4) | ((data[2] & 0xF0) >> 4);
        return compensatePressure(raw) / 100.0;
    }

    // --- Read humidity (%) ---
    public double readHumidityPercent() {
        byte[] data = this.deviceClient.read(Register.HUM_MSB.bVal, 2);
        int raw = ((data[0] & 0xFF) << 8) | (data[1] & 0xFF);
        return compensateHumidity(raw);
    }

    // --- Compensation formulas ---
    private double compensateTemperature(int adc_T) {
        double var1 = ((adc_T / 16384.0) - (dig_T1 / 1024.0)) * dig_T2;
        double var2 = (((adc_T / 131072.0) - (dig_T1 / 8192.0)) * ((adc_T / 131072.0) - (dig_T1 / 8192.0))) * dig_T3;
        t_fine = (int)(var1 + var2);
        return (var1 + var2) / 5120.0;
    }

    private double compensatePressure(int adc_P) {
        double var1 = (t_fine / 2.0) - 64000.0;
        double var2 = var1 * var1 * dig_P6 / 32768.0;
        var2 = var2 + var1 * dig_P5 * 2.0;
        var2 = (var2 / 4.0) + (dig_P4 * 65536.0);
        var1 = (dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1) / 524288.0;
        var1 = (1.0 + var1 / 32768.0) * dig_P1;
        if (var1 == 0.0) return 0; // avoid division by zero
        double p = 1048576.0 - adc_P;
        p = (p - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = dig_P9 * p * p / 2147483648.0;
        var2 = p * dig_P8 / 32768.0;
        return p + (var1 + var2 + dig_P7) / 16.0;
    }

    private double compensateHumidity(int adc_H) {
        double h = t_fine - 76800.0;
        h = (adc_H - (dig_H4 * 64.0 + dig_H5 / 16384.0 * h)) * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * h * (1.0 + dig_H3 / 67108864.0 * h)));
        h = Math.max(0.0, Math.min(100.0, h));
        return h;
    }
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "BME280 Sensor";
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        // No special reset needed; method must be implemented
    }

}