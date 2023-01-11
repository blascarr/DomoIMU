#ifndef _IMU_MPU_H_
#define _IMU_MPU_H_

#include "I2Cdev.h"
#include "utils.h"
#include "MemReg.h"

class IMU_MPU_Base {
    public:

        IMU_MPU_Base(uint8_t address=MPU6050_DEFAULT_ADDRESS);

        void initialize();
        bool testConnection();

        // AUX_VDDIO register
        uint8_t getAuxVDDIOLevel();
        void setAuxVDDIOLevel(uint8_t level);

        // SMPLRT_DIV register
        uint8_t getRate();
        void setRate(uint8_t rate);

        // CONFIG register
        uint8_t getExternalFrameSync();
        void setExternalFrameSync(uint8_t sync);
        uint8_t getDLPFMode();
        void setDLPFMode(uint8_t bandwidth);

        // GYRO_CONFIG register
        uint8_t getFullScaleGyroRange();
        void setFullScaleGyroRange(uint8_t range);

        // SELF_TEST registers
        uint8_t getAccelXSelfTestFactoryTrim();
        uint8_t getAccelYSelfTestFactoryTrim();
        uint8_t getAccelZSelfTestFactoryTrim();

        uint8_t getGyroXSelfTestFactoryTrim();
        uint8_t getGyroYSelfTestFactoryTrim();
        uint8_t getGyroZSelfTestFactoryTrim();

        // ACCEL_CONFIG register
        bool getAccelXSelfTest();
        void setAccelXSelfTest(bool enabled);
        bool getAccelYSelfTest();
        void setAccelYSelfTest(bool enabled);
        bool getAccelZSelfTest();
        void setAccelZSelfTest(bool enabled);
        uint8_t getFullScaleAccelRange();
        void setFullScaleAccelRange(uint8_t range);
        uint8_t getDHPFMode();
        void setDHPFMode(uint8_t mode);

        // FF_THR register
        uint8_t getFreefallDetectionThreshold();
        void setFreefallDetectionThreshold(uint8_t threshold);

        // FF_DUR register
        uint8_t getFreefallDetectionDuration();
        void setFreefallDetectionDuration(uint8_t duration);

        // MOT_THR register
        uint8_t getMotionDetectionThreshold();
        void setMotionDetectionThreshold(uint8_t threshold);

        // MOT_DUR register
        uint8_t getMotionDetectionDuration();
        void setMotionDetectionDuration(uint8_t duration);

        // ZRMOT_THR register
        uint8_t getZeroMotionDetectionThreshold();
        void setZeroMotionDetectionThreshold(uint8_t threshold);

        // ZRMOT_DUR register
        uint8_t getZeroMotionDetectionDuration();
        void setZeroMotionDetectionDuration(uint8_t duration);

        // FIFO_EN register
        bool getTempFIFOEnabled();
        void setTempFIFOEnabled(bool enabled);
        bool getXGyroFIFOEnabled();
        void setXGyroFIFOEnabled(bool enabled);
        bool getYGyroFIFOEnabled();
        void setYGyroFIFOEnabled(bool enabled);
        bool getZGyroFIFOEnabled();
        void setZGyroFIFOEnabled(bool enabled);
        bool getAccelFIFOEnabled();
        void setAccelFIFOEnabled(bool enabled);
        bool getSlave2FIFOEnabled();
        void setSlave2FIFOEnabled(bool enabled);
        bool getSlave1FIFOEnabled();
        void setSlave1FIFOEnabled(bool enabled);
        bool getSlave0FIFOEnabled();
        void setSlave0FIFOEnabled(bool enabled);

        // I2C_MST_CTRL register
        bool getMultiMasterEnabled();
        void setMultiMasterEnabled(bool enabled);
        bool getWaitForExternalSensorEnabled();
        void setWaitForExternalSensorEnabled(bool enabled);
        bool getSlave3FIFOEnabled();
        void setSlave3FIFOEnabled(bool enabled);
        bool getSlaveReadWriteTransitionEnabled();
        void setSlaveReadWriteTransitionEnabled(bool enabled);
        uint8_t getMasterClockSpeed();
        void setMasterClockSpeed(uint8_t speed);

        // I2C_SLV* registers (Slave 0-3)
        uint8_t getSlaveAddress(uint8_t num);
        void setSlaveAddress(uint8_t num, uint8_t address);
        uint8_t getSlaveRegister(uint8_t num);
        void setSlaveRegister(uint8_t num, uint8_t reg);
        bool getSlaveEnabled(uint8_t num);
        void setSlaveEnabled(uint8_t num, bool enabled);
        bool getSlaveWordByteSwap(uint8_t num);
        void setSlaveWordByteSwap(uint8_t num, bool enabled);
        bool getSlaveWriteMode(uint8_t num);
        void setSlaveWriteMode(uint8_t num, bool mode);
        bool getSlaveWordGroupOffset(uint8_t num);
        void setSlaveWordGroupOffset(uint8_t num, bool enabled);
        uint8_t getSlaveDataLength(uint8_t num);
        void setSlaveDataLength(uint8_t num, uint8_t length);

        // I2C_SLV* registers (Slave 4)
        uint8_t getSlave4Address();
        void setSlave4Address(uint8_t address);
        uint8_t getSlave4Register();
        void setSlave4Register(uint8_t reg);
        void setSlave4OutputByte(uint8_t data);
        bool getSlave4Enabled();
        void setSlave4Enabled(bool enabled);
        bool getSlave4InterruptEnabled();
        void setSlave4InterruptEnabled(bool enabled);
        bool getSlave4WriteMode();
        void setSlave4WriteMode(bool mode);
        uint8_t getSlave4MasterDelay();
        void setSlave4MasterDelay(uint8_t delay);
        uint8_t getSlate4InputByte();

        // I2C_MST_STATUS register
        bool getPassthroughStatus();
        bool getSlave4IsDone();
        bool getLostArbitration();
        bool getSlave4Nack();
        bool getSlave3Nack();
        bool getSlave2Nack();
        bool getSlave1Nack();
        bool getSlave0Nack();

        // INT_PIN_CFG register
        bool getInterruptMode();
        void setInterruptMode(bool mode);
        bool getInterruptDrive();
        void setInterruptDrive(bool drive);
        bool getInterruptLatch();
        void setInterruptLatch(bool latch);
        bool getInterruptLatchClear();
        void setInterruptLatchClear(bool clear);
        bool getFSyncInterruptLevel();
        void setFSyncInterruptLevel(bool level);
        bool getFSyncInterruptEnabled();
        void setFSyncInterruptEnabled(bool enabled);
        bool getI2CBypassEnabled();
        void setI2CBypassEnabled(bool enabled);
        bool getClockOutputEnabled();
        void setClockOutputEnabled(bool enabled);

        // INT_ENABLE register
        uint8_t getIntEnabled();
        void setIntEnabled(uint8_t enabled);
        bool getIntFreefallEnabled();
        void setIntFreefallEnabled(bool enabled);
        bool getIntMotionEnabled();
        void setIntMotionEnabled(bool enabled);
        bool getIntZeroMotionEnabled();
        void setIntZeroMotionEnabled(bool enabled);
        bool getIntFIFOBufferOverflowEnabled();
        void setIntFIFOBufferOverflowEnabled(bool enabled);
        bool getIntI2CMasterEnabled();
        void setIntI2CMasterEnabled(bool enabled);
        bool getIntDataReadyEnabled();
        void setIntDataReadyEnabled(bool enabled);

        // INT_STATUS register
        uint8_t getIntStatus();
        bool getIntFreefallStatus();
        bool getIntMotionStatus();
        bool getIntZeroMotionStatus();
        bool getIntFIFOBufferOverflowStatus();
        bool getIntI2CMasterStatus();
        bool getIntDataReadyStatus();

        // ACCEL_*OUT_* registers
        void getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz);
        void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
        void getAcceleration(int16_t* x, int16_t* y, int16_t* z);
        int16_t getAccelerationX();
        int16_t getAccelerationY();
        int16_t getAccelerationZ();

        // TEMP_OUT_* registers
        int16_t getTemperature();

        // GYRO_*OUT_* registers
        void getRotation(int16_t* x, int16_t* y, int16_t* z);
        int16_t getRotationX();
        int16_t getRotationY();
        int16_t getRotationZ();

        // EXT_SENS_DATA_* registers
        uint8_t getExternalSensorByte(int position);
        uint16_t getExternalSensorWord(int position);
        uint32_t getExternalSensorDWord(int position);

        // MOT_DETECT_STATUS register
        uint8_t getMotionStatus();
        bool getXNegMotionDetected();
        bool getXPosMotionDetected();
        bool getYNegMotionDetected();
        bool getYPosMotionDetected();
        bool getZNegMotionDetected();
        bool getZPosMotionDetected();
        bool getZeroMotionDetected();

        // I2C_SLV*_DO register
        void setSlaveOutputByte(uint8_t num, uint8_t data);

        // I2C_MST_DELAY_CTRL register
        bool getExternalShadowDelayEnabled();
        void setExternalShadowDelayEnabled(bool enabled);
        bool getSlaveDelayEnabled(uint8_t num);
        void setSlaveDelayEnabled(uint8_t num, bool enabled);

        // SIGNAL_PATH_RESET register
        void resetGyroscopePath();
        void resetAccelerometerPath();
        void resetTemperaturePath();

        // MOT_DETECT_CTRL register
        uint8_t getAccelerometerPowerOnDelay();
        void setAccelerometerPowerOnDelay(uint8_t delay);
        uint8_t getFreefallDetectionCounterDecrement();
        void setFreefallDetectionCounterDecrement(uint8_t decrement);
        uint8_t getMotionDetectionCounterDecrement();
        void setMotionDetectionCounterDecrement(uint8_t decrement);

        // USER_CTRL register
        bool getFIFOEnabled();
        void setFIFOEnabled(bool enabled);
        bool getI2CMasterModeEnabled();
        void setI2CMasterModeEnabled(bool enabled);
        void switchSPIEnabled(bool enabled);
        void resetFIFO();
        void resetI2CMaster();
        void resetSensors();

        // PWR_MGMT_1 register
        void reset();
        bool getSleepEnabled();
        void setSleepEnabled(bool enabled);
        bool getWakeCycleEnabled();
        void setWakeCycleEnabled(bool enabled);
        bool getTempSensorEnabled();
        void setTempSensorEnabled(bool enabled);
        uint8_t getClockSource();
        void setClockSource(uint8_t source);

        // PWR_MGMT_2 register
        uint8_t getWakeFrequency();
        void setWakeFrequency(uint8_t frequency);
        bool getStandbyXAccelEnabled();
        void setStandbyXAccelEnabled(bool enabled);
        bool getStandbyYAccelEnabled();
        void setStandbyYAccelEnabled(bool enabled);
        bool getStandbyZAccelEnabled();
        void setStandbyZAccelEnabled(bool enabled);
        bool getStandbyXGyroEnabled();
        void setStandbyXGyroEnabled(bool enabled);
        bool getStandbyYGyroEnabled();
        void setStandbyYGyroEnabled(bool enabled);
        bool getStandbyZGyroEnabled();
        void setStandbyZGyroEnabled(bool enabled);

        // FIFO_COUNT_* registers
        uint16_t getFIFOCount();

        // FIFO_R_W register
        uint8_t getFIFOByte();
		int8_t GetCurrentFIFOPacket(uint8_t *data, uint8_t length);
        void setFIFOByte(uint8_t data);
        void getFIFOBytes(uint8_t *data, uint8_t length);

        // WHO_AM_I register
        uint8_t getDeviceID();
        void setDeviceID(uint8_t id);
        
        // ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========
        
        // XG_OFFS_TC register
        uint8_t getOTPBankValid();
        void setOTPBankValid(bool enabled);
        int8_t getXGyroOffsetTC();
        void setXGyroOffsetTC(int8_t offset);

        // YG_OFFS_TC register
        int8_t getYGyroOffsetTC();
        void setYGyroOffsetTC(int8_t offset);

        // ZG_OFFS_TC register
        int8_t getZGyroOffsetTC();
        void setZGyroOffsetTC(int8_t offset);

        // X_FINE_GAIN register
        int8_t getXFineGain();
        void setXFineGain(int8_t gain);

        // Y_FINE_GAIN register
        int8_t getYFineGain();
        void setYFineGain(int8_t gain);

        // Z_FINE_GAIN register
        int8_t getZFineGain();
        void setZFineGain(int8_t gain);

        // XA_OFFS_* registers
        int16_t getXAccelOffset();
        void setXAccelOffset(int16_t offset);

        // YA_OFFS_* register
        int16_t getYAccelOffset();
        void setYAccelOffset(int16_t offset);

        // ZA_OFFS_* register
        int16_t getZAccelOffset();
        void setZAccelOffset(int16_t offset);

        // XG_OFFS_USR* registers
        int16_t getXGyroOffset();
        void setXGyroOffset(int16_t offset);

        // YG_OFFS_USR* register
        int16_t getYGyroOffset();
        void setYGyroOffset(int16_t offset);

        // ZG_OFFS_USR* register
        int16_t getZGyroOffset();
        void setZGyroOffset(int16_t offset);
        
        // INT_ENABLE register (DMP functions)
        bool getIntPLLReadyEnabled();
        void setIntPLLReadyEnabled(bool enabled);
        bool getIntDMPEnabled();
        void setIntDMPEnabled(bool enabled);
        
        // DMP_INT_STATUS
        bool getDMPInt5Status();
        bool getDMPInt4Status();
        bool getDMPInt3Status();
        bool getDMPInt2Status();
        bool getDMPInt1Status();
        bool getDMPInt0Status();

        // INT_STATUS register (DMP functions)
        bool getIntPLLReadyStatus();
        bool getIntDMPStatus();
        
        // USER_CTRL register (DMP functions)
        bool getDMPEnabled();
        void setDMPEnabled(bool enabled);
        void resetDMP();
        
        // BANK_SEL register
        void setMemoryBank(uint8_t bank, bool prefetchEnabled=false, bool userBank=false);
        
        // MEM_START_ADDR register
        void setMemoryStartAddress(uint8_t address);
        
        // MEM_R_W register
        uint8_t readMemoryByte();
        void writeMemoryByte(uint8_t data);
        void readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank=0, uint8_t address=0);
        bool writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank=0, uint8_t address=0, bool verify=true, bool useProgMem=false);
        bool writeProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank=0, uint8_t address=0, bool verify=true);

        bool writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize, bool useProgMem=false);
        bool writeProgDMPConfigurationSet(const uint8_t *data, uint16_t dataSize);

        // DMP_CFG_1 register
        uint8_t getDMPConfig1();
        void setDMPConfig1(uint8_t config);

        // DMP_CFG_2 register
        uint8_t getDMPConfig2();
        void setDMPConfig2(uint8_t config);

		// Calibration Routines
		void CalibrateGyro(uint8_t Loops = 15); // Fine tune after setting offsets with less Loops.
		void CalibrateAccel(uint8_t Loops = 15);// Fine tune after setting offsets with less Loops.
		void PID(uint8_t ReadAddress, float kP,float kI, uint8_t Loops);  // Does the math
		void PrintActiveOffsets(); // See the results of the Calibration

    private:
        uint8_t devAddr;
        uint8_t buffer[14];  
};

#ifndef I2CDEVLIB_MPU6050_TYPEDEF
#define I2CDEVLIB_MPU6050_TYPEDEF
    typedef IMU_MPU_Base IMU_MPU;
#endif

#endif /* _IMU_MPU_H_ */

