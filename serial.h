// Copyright Alten AB, Linköping 2015
//
// Authors: Sören Molander, Joel Nordh, Pierre Östlund
//

#ifndef __SERIAL_HH__
#define __SERIAL_HH__

#include <pthread.h>

class ArduinoSerial {
public:
    ArduinoSerial();
    virtual ~ArduinoSerial();
    
    static const int LEFT_MOTOR = 0;
    static const int RIGHT_MOTOR = 1;
    int engine_forward(unsigned engine, unsigned speed);
    int engine_backwards(int engine, unsigned speed);
    int engine_halt(int engine);
    int servo_set_position(int servo, unsigned position);

    int write_to_serial(const char* str);
    int send_command(const char* format, ...);

    void get_mpu_data(int* output);

private:
    void open_port();
    void init_reader();

    struct MpuData {
      int values[6]; // AcX, AcY, AcZ,  GcX, GcY, GcZ
      int fd;
      pthread_mutex_t mutex;
    };

    static void* reader_thread(void* arg);

    int* fd;
    pthread_t reader;
    MpuData mpuData;
};

#endif
