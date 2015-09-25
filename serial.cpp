// Copyright Alten AB, Linköping 2015
//
// Authors: Pierre Östlund
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdarg.h>
#include <pthread.h>
#include "serial.h"

#define END_FILE_CHARACTER 0x04
#define SERIAL_PORT "/dev/ttyACM0"

ArduinoSerial::ArduinoSerial() : fd(0) {
  open_port();
  init_reader();
}

void ArduinoSerial::open_port() {
  struct termios serial;

  printf("[Serial] Opening %s\n", SERIAL_PORT);

  fd = (int*)malloc(sizeof(*fd));
  *fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
  if (*fd == -1) {
    perror(SERIAL_PORT);
    throw "Failed to open port";
  }

  if (tcgetattr(*fd, &serial) < 0) {
    close(*fd);
    perror("Getting configuration");
    throw "Failed to get configuration";
  }

  serial.c_cflag = B115200 | CS8 | CREAD;
  serial.c_iflag = IGNPAR | ICRNL;
  serial.c_oflag = 0;
  serial.c_lflag = ICANON;
  tcsetattr(*fd, TCSANOW, &serial);
}

void ArduinoSerial::init_reader() {
  mpuData.fd = *fd;
  pthread_mutex_init(&mpuData.mutex, NULL);
  if (pthread_create(&reader, NULL, reader_thread, (void*)&mpuData) != 0) {
    throw "Failed to create reader thread";
  }
}

void* ArduinoSerial::reader_thread(void* arg) {
  char buffer[1024];
  char* p = buffer;
  MpuData* data = (MpuData *)arg;
  
  for (;;) {
    int n = read(data->fd, p, 100);
    if (n > 0) {
      if (*p == '\n' || *p == '\r') {
	int AcX, AcY, AcZ, tmp, GcX, GcY, GcZ;
	if(sscanf(buffer, "mpu %u %u %u %u %u %u %u",
		  &AcX, &AcY, &AcZ, &GcX, &GcY, &GcZ) == 7) {
	// if(sscanf(buffer, "mpu %u %u %u %u %u %u %u",
	// 	  &AcX, &AcY, &AcZ, &tmp, &GcX, &GcY, &GcZ) == 7) {
/* FIXME: Optimize, lock time to keep lock here */
	  pthread_mutex_lock(&data->mutex);
	  data->values[0] = AcX;
	  data->values[1] = AcY;
	  data->values[2] = AcZ;
	  // data->values[3] = tmp;
	  data->values[3] = GcX;
	  data->values[4] = GcY;
	  data->values[5] = GcZ;
	  pthread_mutex_unlock(&data->mutex);
	}
	p = buffer;
      } else {
	p += n;
      }
    } else if (n != 0) {
      printf("Failed to read from UART: %d\n", n);
    }
  }
}

void ArduinoSerial::get_mpu_data(int* output) {
  pthread_mutex_lock(&mpuData.mutex);
  memcpy(output, mpuData.values, sizeof(mpuData.values));
  pthread_mutex_unlock(&mpuData.mutex);
}

ArduinoSerial::~ArduinoSerial() {
  pthread_cancel(reader);
  close(*fd);
  free(fd);
}

int ArduinoSerial::write_to_serial(const char* str) {
  int wcount = write(*fd, str, strlen(str));
  if (wcount < 0) {
    close(*fd);
    perror("Write");
    return -1;
  }
  return wcount;
}

int ArduinoSerial::send_command(const char* format, ...) {
  char buf[30];
	
  va_list args;
  va_start(args, format);
  vsprintf(buf, format, args);
  va_end(args);
	
  printf("[Serial] Send: %s", buf);
  return write_to_serial(buf);
}

int ArduinoSerial::engine_forward(unsigned engine, unsigned speed) {
  return send_command("f%u %u\n", engine, speed);
}

int ArduinoSerial::engine_backwards(int engine, unsigned speed) {
  return send_command("b%u %u\n", engine, speed);
}

int ArduinoSerial::engine_halt(int engine) {
  return send_command("f%u\n", engine);
}

int ArduinoSerial::servo_set_position(int servo, unsigned position) {
  return send_command("s%u %u\n", servo, position);
}

// char linux_getch(void) {
//   struct termios oldstuff;
//   struct termios newstuff;
//   int inch;

//   tcgetattr(STDIN_FILENO, &oldstuff);
//   newstuff = oldstuff; /* save old attributes */
//   newstuff.c_lflag &= ~(ICANON | ECHO); /* reset "canonical" and "echo" flags*/
//   tcsetattr(STDIN_FILENO, TCSANOW, &newstuff); /* set new attributes */
//   inch = getchar();
//   tcsetattr(STDIN_FILENO, TCSANOW, &oldstuff); /* restore old attributes */
//   if (inch == END_FILE_CHARACTER) {
//     inch = EOF;
//   }
//   return (char)inch;
// }

// int main(int argc, char* argv[]) {
//   //printf("ret: %d\n", ArduinoSerial::engine_forward(0, 100));
//   int c, step = 0;
//   ArduinoSerial serial;
//   while(c =linux_getch()) {
//     if (step == 0 && c == 27) {
//       step++;
//     } else if (step == 1 && c == 91) {
//       step++;
//     } else if (step == 2 && c == 65) { /* UP */
//       serial.engine_backwards(0, 255);
//       serial.engine_forward(1, 255);
//     } else if (step == 2 && c == 66) { /* Down */
//       serial.engine_forward(0, 255);
//       serial.engine_backwards(1, 255);
//     } else if (step == 2 && c == 67) { /* Right */
//       serial.engine_backwards(0, 100);
//       serial.engine_backwards(1, 100);
//     } else if (step == 2 && c == 68) { /* Left */
//       serial.engine_forward(0, 100);
//       serial.engine_forward(1, 100);
//     } else if (c == 'h') {
//       step = 0;
//       serial.engine_halt(0);
//       serial.engine_halt(1);
//     } else if (c == 'm') {
//       int mpuData[7];
//       serial.get_mpu_data(mpuData);
//       printf("AcX: %d, AcY: %d, AcZ: %d, Temp: %d, GcX: %d, GcY: %d, GcZ: %d\n",
// 	     mpuData[0], mpuData[1], mpuData[2], mpuData[3], mpuData[4], mpuData[5], mpuData[6]);
//     } else {
//       step = 0;
//     }
//   }

//   return 0;
// }
