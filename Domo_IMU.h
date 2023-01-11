#ifndef _Domo_IMU_H_
    #define _Domo_IMU_H_
    #include "IMU_MPU.h"
    #include "utils.h"

    //#define DEBUG
    #ifdef DEBUG
        #define DEBUG_PRINT(x) Serial.print(x)
        #define DEBUG_PRINTF(x, y) Serial.print(x, y)
        #define DEBUG_PRINTLN(x) Serial.println(x)
        #define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
    #else
        #define DEBUG_PRINT(x)
        #define DEBUG_PRINTF(x, y)
        #define DEBUG_PRINTLN(x)
        #define DEBUG_PRINTLNF(x, y)
    #endif

    class Domo_IMU : public IMU_MPU {
    public:
        // Interruption Mode
        uint8_t IMU_INTERRUPT_PIN;
        bool IMU_INTERRUPT_MODE = false;
        typedef void ( Domo_IMU::*_f_update )();
        _f_update controller = &Domo_IMU::read; //Loop Controller Function

        // MPU control/status vars
        bool dmpReady = false;  // set true if DMP init was successful
        uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
        uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
        uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
        uint16_t fifoCount;     // count of all bytes currently in FIFO
        uint8_t fifoBuffer[64]; // FIFO storage buffer
        float calibrationAccel;
        float calibrationGyro;

        // orientation/motion vars
        Quaternion q;           // [w, x, y, z]         quaternion container
        VectorInt16 aa;         // [x, y, z]            accel sensor measurements
        VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
        VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
        VectorFloat gravity;    // [x, y, z]            gravity vector
        VectorFloat gyroOffsets;    // [x, y, z]        gyro Offset vector
        VectorFloat accelOffsets;   // [x, y, z]        Accel Offset vector
        float euler[3];         // [psi, theta, phi]    Euler angle container
        float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

        void ( *_f_interrupt )();
        
        Domo_IMU( uint8_t address=MPU6050_DEFAULT_ADDRESS ): IMU_MPU( address ){
        
        }

        Domo_IMU( uint16_t interrupt_pin, void (*fn)(), uint8_t address=MPU6050_DEFAULT_ADDRESS ): IMU_INTERRUPT_PIN( interrupt_pin ), IMU_MPU( address ){
            DEBUG_PRINTLN(F("INTERRUPT MODE..."));
            IMU_INTERRUPT_MODE = true;
            _f_interrupt = fn;
            controller = &Domo_IMU::interrupt_read;
        }

        Domo_IMU(uint8_t address=MPU6050_DEFAULT_ADDRESS, void *wireObj=0) : IMU_MPU(address) { }

        void setAccelOffsets( float XAccelOffset, float YAccelOffset, float ZAccelOffset ){
            accelOffsets.x = XAccelOffset;
            accelOffsets.y = YAccelOffset;
            accelOffsets.z = ZAccelOffset;
        }
        void setGyroOffsets( float XGyroOffset, float YGyroOffset, float ZGyroOffset ){
            gyroOffsets.x = XGyroOffset;
            gyroOffsets.y = YGyroOffset;
            gyroOffsets.z = ZGyroOffset;
        }

        void calibrateAcc( float calAccel ) {
            calibrationAccel = calAccel;
        }

        void calibrateGyro( float calGyro ) {
            calibrationGyro = calGyro;
        }

        void calibrate( float calAccel, float calGyro ) {
            calibrationAccel = calAccel;
            calibrationGyro = calGyro;
        }

        void interruptOff(){
            IMU_INTERRUPT_MODE = false;
            detachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN));
        }

        void interruptOn(){
            IMU_INTERRUPT_MODE = true;
            attachInterrupt(digitalPinToInterrupt( IMU_INTERRUPT_PIN ), _f_interrupt, RISING);
        }

        void init(){
            #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
                Wire.begin();
                Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
            #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
                Fastwire::setup(400, true);
            #endif
            
            DEBUG_PRINTLN(F("Initializing I2C devices..."));
            initialize();
            if( IMU_INTERRUPT_MODE ){
                DEBUG_PRINTLN(F("Interrupt ON"));
                pinMode( IMU_INTERRUPT_PIN, INPUT);
                delay(300);
            }else{
                DEBUG_PRINTLN(F("Interrupt OFF"));
            }
            
            // verify connection
            DEBUG_PRINTLN(F("Testing device connections..."));
            DEBUG_PRINTLN( testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

            // load and configure the DMP
            DEBUG_PRINTLN(F("Initializing DMP..."));
            devStatus = dmpInitialize();

            setXGyroOffset( gyroOffsets.x );
            setYGyroOffset( gyroOffsets.y );
            setZGyroOffset( gyroOffsets.z );
            
            //setXAccelOffset( accelOffsets.x );
            //setYAccelOffset( accelOffsets.y );
            setZAccelOffset( accelOffsets.z );

            // make sure it worked (returns 0 if so)
            if (devStatus == 0) {
                // Calibration Time: generate offsets and calibrate our MPU6050
                CalibrateAccel( calibrationAccel );
                CalibrateGyro( calibrationGyro );
                PrintActiveOffsets();
                // turn on the DMP, now that it's ready
                DEBUG_PRINTLN(F("Enabling DMP..."));
                setDMPEnabled(true);

                // enable Arduino interrupt detection
                if( IMU_INTERRUPT_MODE ){
                    DEBUG_PRINT(F("Enabling interrupt detection (Arduino external interrupt "));
                    DEBUG_PRINT(digitalPinToInterrupt( IMU_INTERRUPT_PIN ));
                    DEBUG_PRINTLN(F(")..."));
                    attachInterrupt(digitalPinToInterrupt( IMU_INTERRUPT_PIN ), _f_interrupt, RISING);
                    mpuIntStatus = getIntStatus();

                    // set our DMP Ready flag so the main loop() function knows it's okay to use it
                    dmpReady = true;
                }
                // get expected DMP packet size for later comparison
                packetSize = dmpGetFIFOPacketSize();
            } else {
                // ERROR!
                // 1 = initial memory load failed
                // 2 = DMP configuration updates failed
                // (if it's going to break, usually the code will be 1)
                DEBUG_PRINT(F("DMP Initialization failed (code "));
                DEBUG_PRINT(devStatus);
                DEBUG_PRINTLN(F(")"));
            }
        }

        void read(){
            // read a packet from FIFO
            if (!dmpReady) return;
            if ( dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
                    
                // display Euler angles in degrees
                dmpGetQuaternion(&q, fifoBuffer);
                dmpGetGravity(&gravity, &q);
                dmpGetYawPitchRoll(ypr, &q, &gravity);
            }
        }

        void interrupt_read(){
            read();
        }
        
        void update(){
            (this->*controller)();
        }

        void idle(){}

        uint8_t dmpInitialize();
        bool dmpPacketAvailable();

        uint8_t dmpSetFIFORate(uint8_t fifoRate);
        uint8_t dmpGetFIFORate();
        uint8_t dmpGetSampleStepSizeMS();
        uint8_t dmpGetSampleFrequency();
        int32_t dmpDecodeTemperature(int8_t tempReg);
        
        // Register callbacks after a packet of FIFO data is processed
        //uint8_t dmpRegisterFIFORateProcess(inv_obj_func func, int16_t priority);
        //uint8_t dmpUnregisterFIFORateProcess(inv_obj_func func);
        uint8_t dmpRunFIFORateProcesses();
        
        // Setup FIFO for various output
        uint8_t dmpSendQuaternion(uint_fast16_t accuracy);
        uint8_t dmpSendGyro(uint_fast16_t elements, uint_fast16_t accuracy);
        uint8_t dmpSendAccel(uint_fast16_t elements, uint_fast16_t accuracy);
        uint8_t dmpSendLinearAccel(uint_fast16_t elements, uint_fast16_t accuracy);
        uint8_t dmpSendLinearAccelInWorld(uint_fast16_t elements, uint_fast16_t accuracy);
        uint8_t dmpSendControlData(uint_fast16_t elements, uint_fast16_t accuracy);
        uint8_t dmpSendSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
        uint8_t dmpSendExternalSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
        uint8_t dmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
        uint8_t dmpSendPacketNumber(uint_fast16_t accuracy);
        uint8_t dmpSendQuantizedAccel(uint_fast16_t elements, uint_fast16_t accuracy);
        uint8_t dmpSendEIS(uint_fast16_t elements, uint_fast16_t accuracy);

        // Get Fixed Point data from FIFO
        uint8_t dmpGetAccel(int32_t *data, const uint8_t* packet=0);
        uint8_t dmpGetAccel(int16_t *data, const uint8_t* packet=0);
        uint8_t dmpGetAccel(VectorInt16 *v, const uint8_t* packet=0);
        uint8_t dmpGetQuaternion(int32_t *data, const uint8_t* packet=0);
        uint8_t dmpGetQuaternion(int16_t *data, const uint8_t* packet=0);
        uint8_t dmpGetQuaternion(Quaternion *q, const uint8_t* packet=0);
        uint8_t dmpGet6AxisQuaternion(int32_t *data, const uint8_t* packet=0);
        uint8_t dmpGet6AxisQuaternion(int16_t *data, const uint8_t* packet=0);
        uint8_t dmpGet6AxisQuaternion(Quaternion *q, const uint8_t* packet=0);
        uint8_t dmpGetRelativeQuaternion(int32_t *data, const uint8_t* packet=0);
        uint8_t dmpGetRelativeQuaternion(int16_t *data, const uint8_t* packet=0);
        uint8_t dmpGetRelativeQuaternion(Quaternion *data, const uint8_t* packet=0);
        uint8_t dmpGetGyro(int32_t *data, const uint8_t* packet=0);
        uint8_t dmpGetGyro(int16_t *data, const uint8_t* packet=0);
        uint8_t dmpGetGyro(VectorInt16 *v, const uint8_t* packet=0);
        uint8_t dmpSetLinearAccelFilterCoefficient(float coef);
        uint8_t dmpGetLinearAccel(int32_t *data, const uint8_t* packet=0);
        uint8_t dmpGetLinearAccel(int16_t *data, const uint8_t* packet=0);
        uint8_t dmpGetLinearAccel(VectorInt16 *v, const uint8_t* packet=0);
        uint8_t dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
        uint8_t dmpGetLinearAccelInWorld(int32_t *data, const uint8_t* packet=0);
        uint8_t dmpGetLinearAccelInWorld(int16_t *data, const uint8_t* packet=0);
        uint8_t dmpGetLinearAccelInWorld(VectorInt16 *v, const uint8_t* packet=0);
        uint8_t dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
        uint8_t dmpGetGyroAndAccelSensor(int32_t *data, const uint8_t* packet=0);
        uint8_t dmpGetGyroAndAccelSensor(int16_t *data, const uint8_t* packet=0);
        uint8_t dmpGetGyroAndAccelSensor(VectorInt16 *g, VectorInt16 *a, const uint8_t* packet=0);
        uint8_t dmpGetGyroSensor(int32_t *data, const uint8_t* packet=0);
        uint8_t dmpGetGyroSensor(int16_t *data, const uint8_t* packet=0);
        uint8_t dmpGetGyroSensor(VectorInt16 *v, const uint8_t* packet=0);
        uint8_t dmpGetControlData(int32_t *data, const uint8_t* packet=0);
        uint8_t dmpGetTemperature(int32_t *data, const uint8_t* packet=0);
        uint8_t dmpGetGravity(int32_t *data, const uint8_t* packet=0);
        uint8_t dmpGetGravity(int16_t *data, const uint8_t* packet=0);
        uint8_t dmpGetGravity(VectorInt16 *v, const uint8_t* packet=0);
        uint8_t dmpGetGravity(VectorFloat *v, Quaternion *q);
        uint8_t dmpGetUnquantizedAccel(int32_t *data, const uint8_t* packet=0);
        uint8_t dmpGetUnquantizedAccel(int16_t *data, const uint8_t* packet=0);
        uint8_t dmpGetUnquantizedAccel(VectorInt16 *v, const uint8_t* packet=0);
        uint8_t dmpGetQuantizedAccel(int32_t *data, const uint8_t* packet=0);
        uint8_t dmpGetQuantizedAccel(int16_t *data, const uint8_t* packet=0);
        uint8_t dmpGetQuantizedAccel(VectorInt16 *v, const uint8_t* packet=0);
        uint8_t dmpGetExternalSensorData(int32_t *data, uint16_t size, const uint8_t* packet=0);
        uint8_t dmpGetEIS(int32_t *data, const uint8_t* packet=0);
        
        uint8_t dmpGetEuler(float *data, Quaternion *q);
        uint8_t dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);

        // Get Floating Point data from FIFO
        uint8_t dmpGetAccelFloat(float *data, const uint8_t* packet=0);
        uint8_t dmpGetQuaternionFloat(float *data, const uint8_t* packet=0);

        uint8_t dmpProcessFIFOPacket(const unsigned char *dmpData);
        uint8_t dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed=NULL);

        uint8_t dmpSetFIFOProcessedCallback(void (*func) (void));

        uint8_t dmpInitFIFOParam();
        uint8_t dmpCloseFIFO();
        uint8_t dmpSetGyroDataSource(uint8_t source);
        uint8_t dmpDecodeQuantizedAccel();
        uint32_t dmpGetGyroSumOfSquare();
        uint32_t dmpGetAccelSumOfSquare();
        void dmpOverrideQuaternion(long *q);
        uint16_t dmpGetFIFOPacketSize();
        uint8_t dmpGetCurrentFIFOPacket(uint8_t *data); // overflow proof

        private:
            uint8_t *dmpPacketBuffer;
            uint16_t dmpPacketSize;
    };

#endif /* _Domo_IMU_H_ */