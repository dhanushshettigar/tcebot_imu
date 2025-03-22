# I2C Setup on Ubuntu for BNO055 Sensor

## For Ubuntu on Raspberry Pi:

### 1.Check if I2C is enabled:

```
ls /dev/i2c-*
```

If you see output like /dev/i2c-1, I2C is enabled.

### 2. Enable I2C manually (Ubuntu does not have raspi-config):Edit the boot configuration file:
```
sudo nano /boot/firmware/config.txt
```

Add the following lines at the end:
```
dtparam=i2c_arm=on
```

Save and exit (CTRL + X, then Y, then ENTER).

### 3. Check I2C Device Connection

After connecting the BNO055 sensor to the I2C pins, run:
```
sudo i2cdetect -y 1
```
You should see an address like 0x28 or 0x29, confirming the sensor is detected.

### 4. Install Required Python Libraries

Ensure the necessary Python libraries are installed:
```
pip install adafruit-circuitpython-bno055 --break-system-packages
```
Once completed, your BNO055 sensor should be accessible via I2C in your tcebot_imu package.
