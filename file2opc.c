/** \file
 *  OPC image packet reader.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <inttypes.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <getopt.h>
#include <linux/input.h>
#include "util.h"

#define FALSE 0
#define TRUE 1

#define HOST_NAME_MAX 255
#define DEV_NAME_MAX 255

#define EV_PRESSED 1 
#define EV_RELEASED 0 
#define EV_REPEAT 2 

typedef struct
{
  pthread_t ev_t;
  int fd;
  float brightness_target;
  float brightness;
  int frame_rate;
  int base_frame_rate;
  int mode;
  int write_delay;
} ev_data_t;

typedef struct
{
  uint8_t channel;
  uint8_t command;
  uint8_t len_hi;
  uint8_t len_lo;
} opc_cmd_t;

void print_usage()
{
  fprintf(stderr, "Usage: file2opc -r <input file> [-h <ipaddr> (127.0.0.1)]\n" \
    "                                [-p <tcp port> (7890)]\n" \
    "                                [-u <usb device>]\n" \
    "                                [-f <frame rate> (30fps)]\n" \
    "                                [-a 0|1 (0)] (random start) \n" \
    "                                [-l 0|1 (1)] (loop)\n" \
    "                                [-R <remote event device>]\n" \
    "                                [-L 0|1 (0)] enable LIRC\n\n");
}


int fileSize(int fd) {
  struct stat s;
  if (fstat(fd, &s) == -1) {
    int saveErrno = errno;
    fprintf(stderr, "fstat(%d) returned errno=%d.", fd, saveErrno);
    return(-1);
  }
  return(s.st_size);
}


static int
tcp_socket(
const char* host,
const int port
)
{
  const int sock = socket(AF_INET, SOCK_STREAM, 0);
  struct sockaddr_in addr = {
    .sin_family = AF_INET,
    .sin_port = htons(port)
  };
  inet_pton(AF_INET, host, &addr.sin_addr.s_addr);

  if (sock < 0)
    return -1;
  if (connect(sock, (const struct sockaddr*) &addr, sizeof(addr)) < 0)
    return -1;

  return sock;
}

static int
unix_socket(
const char* path)
{
  const int sock = socket(AF_UNIX, SOCK_STREAM, 0);
  struct sockaddr_un addr;
  addr.sun_family = AF_UNIX;
  strcpy(addr.sun_path, path);

  if (sock < 0)
    return -1;
  if (connect(sock, (const struct sockaddr*) &addr, sizeof(addr)) < 0)
    return -1;

  return sock;
}


void* event_thread(void* arg)
{
  struct input_event event;
  ev_data_t* ev_data = (ev_data_t*)arg;

  while (TRUE)
  {
    read(ev_data->fd, &event, sizeof(struct input_event));
    if (event.type == EV_KEY)
    {
      if (event.value == EV_PRESSED || event.value == EV_REPEAT)
      {
        //printf("%d\n", event.code);
        switch (event.code)
        {
        case 59: // "power"
          if (ev_data->mode != 0)
            printf(" OFF \n");
          ev_data->mode = 0;
          break;

        case 60: // "A"
        case 61: // "B"
        case 62: // "C"
          if (ev_data->mode != 1)
            printf(" ON \n");
          ev_data->mode = 1;
          break;

        case 63: // "O"
          ev_data->frame_rate = ev_data->base_frame_rate;
          ev_data->brightness_target = 1.0;
          break;

        case 77: // "right"
          if (ev_data->frame_rate < ev_data->base_frame_rate*1.5)
            ev_data->frame_rate++;
          printf(" Frame rate: %d fps\n", ev_data->frame_rate);
          break;

        case 75: // "left"
          if (ev_data->frame_rate > ev_data->base_frame_rate*0.5)
            ev_data->frame_rate--;
          printf(" Frame rate: %d fps\n", ev_data->frame_rate);
          break;

        case 72: // "up"
          ev_data->brightness_target += 0.01;
          if (ev_data->brightness_target > 1.0)
            ev_data->brightness_target = 1.0;

          printf(" Brightness: %0.2f \n", ev_data->brightness_target);

          break;

        case 80: // "down"
          ev_data->brightness_target -= 0.01;
          if (ev_data->brightness_target < 0.2)
            ev_data->brightness_target = 0.2;

          printf(" Brightness: %0.2f \n", ev_data->brightness_target);

          break;

        default:
          printf(" Remote code: %d\n", event.code);
          break;
        }

        ev_data->write_delay = 100;
      }
    }
  }
}

void* lirc_thread(void* arg)
{
  ev_data_t* ev_data = (ev_data_t*)arg;
  char buf[128];
  char* part;

  while (TRUE)
  {
    int bytes_read = read(ev_data->fd, buf, 128);
    if (bytes_read <= 0)
      continue;

    part = strtok(buf, "_");
    part = strtok(NULL, " ");

    if (strcmp(part, "F1") == 0) // "power"
    {
      if (ev_data->mode != 0)
        printf(" OFF \n");
      ev_data->mode = 0;
    }
    else if (strcmp(part, "F2") == 0 || // "A"
             strcmp(part, "F3") == 0 || // "B"
             strcmp(part, "F4") == 0)   // "C"
    {
      if (ev_data->mode != 1)
        printf(" ON \n");
      ev_data->mode = 1;
    }
    else if (strcmp(part, "KP5") == 0) // "O"
    {
          ev_data->frame_rate = ev_data->base_frame_rate;
          ev_data->brightness_target = 1.0;
    }
    else if (strcmp(part, "KP6") == 0) // "right"
    {
      if (ev_data->frame_rate < ev_data->base_frame_rate*1.5)
        ev_data->frame_rate++;
      printf(" Frame rate: %d fps\n", ev_data->frame_rate);
    }
    else if (strcmp(part, "KP4") == 0) // "left"
    {
      if (ev_data->frame_rate > ev_data->base_frame_rate*0.5)
        ev_data->frame_rate--;
      printf(" Frame rate: %d fps\n", ev_data->frame_rate);
    }
    else if (strcmp(part, "KP8") == 0) // "up"
    {
      ev_data->brightness_target += 0.01;
      if (ev_data->brightness_target > 1.0)
        ev_data->brightness_target = 1.0;

      printf(" Brightness: %0.2f \n", ev_data->brightness_target);
    }
    else if (strcmp(part, "KP2") == 0) // "down"
    {
      ev_data->brightness_target -= 0.01;
      if (ev_data->brightness_target < 0.2)
        ev_data->brightness_target = 0.2;

      printf(" Brightness: %0.2f \n", ev_data->brightness_target);
    }
    else
      printf("Received unknown LIRC KEY_%s\n", part);

    ev_data->write_delay = 100;
  }
}

int
main(
int argc,
char ** argv
)
{
  int port = 7890;
  int frame_rate = 30;

  extern char *optarg;
  int opt;

  int fd = 0, fout = 0;
  int loop = TRUE;
  char host[HOST_NAME_MAX + 1];
  strncpy(host, "127.0.0.1", HOST_NAME_MAX);

  char usb[DEV_NAME_MAX + 1];
  memset(usb, 0, DEV_NAME_MAX + 1);

  char remote[DEV_NAME_MAX + 1];
  memset(remote, 0, DEV_NAME_MAX + 1);

  int randomStart = FALSE;

  int lirc = FALSE;
  int fd_lirc = 0;

  fprintf(stderr, "OpenPixelControl File Reader\nMads Christensen (c) 2015-2018, www.madschristensen.info\n\n");

  while ((opt = getopt(argc, argv, "h:p:u:r:l:a:f:R:L:")) != -1)
  {
    switch (opt)
    {
    case 'h':
      strncpy(host, optarg, HOST_NAME_MAX);
      break;

    case 'p':
      port = atoi(optarg);
      break;

    case 'r':
      fd = open(optarg, O_RDONLY);
      if (fd < 0)
        printf("Cannot open input file: %s\n", optarg);
      else
        printf("Reading OPC data from: %s\n", optarg);

      break;

    case 'l':
      loop = atoi(optarg);
      break;

    case 'a':
      randomStart = atoi(optarg);
      break;

    case 'f':
      frame_rate = atoi(optarg);
      break;

    case 'R':
      strncpy(remote, optarg, DEV_NAME_MAX);
      break;

    case 'L':
      lirc = TRUE;
      break;

    case 'u':
      strncpy(usb, optarg, DEV_NAME_MAX);
      break;


    default:
      print_usage();
      exit(EXIT_FAILURE);
    }
  }

  if (fd == 0)
  {
    print_usage();
    die("Input file must be specified\n");
  }

  ev_data_t ev_data;
  memset(&ev_data, 0, sizeof(ev_data_t));
  ev_data.mode = 1; // start on
  ev_data.brightness_target = 1.0; // full brightness
  ev_data.frame_rate = ev_data.base_frame_rate = frame_rate;

  // Read saved settings
  char cfg[DEV_NAME_MAX+1];
  strncpy(cfg, argv[0], DEV_NAME_MAX-4);
  strcat(cfg, ".cfg");
  int fd_cfg = open(cfg, O_RDONLY);
  if (fd_cfg > 0)
  {
    printf("Reading settings from: %s\n", cfg);
    float brightness;
    read(fd_cfg, &brightness, sizeof(brightness));
    if (brightness < 0.2 || brightness > 1.0)
      brightness = 1.0;
    ev_data.brightness_target = brightness;

    int fr;
    read(fd_cfg, &fr, sizeof(fr));
    if (fr < 0.5*frame_rate || fr > 1.5*frame_rate)
      fr = frame_rate;
    ev_data.frame_rate = fr;

    close(fd_cfg);
  }

  int fd_out = -1;

  // Open connection to remote receiver
  if (remote[0] != 0)
  {
    if ((ev_data.fd = open(remote, O_RDONLY)) < 0)
      printf("Error opening remote event device %s\n", remote);
    else
    {
      printf("Listening for input events on: %s\n", remote);
      pthread_create(&ev_data.ev_t, NULL, &event_thread, &ev_data);
    }
  }
  else if (lirc)
  {
    if ((ev_data.fd = unix_socket("/var/run/lirc/lircd")) < 0)
      printf("Connect to lircd failed: %s\n", strerror(errno));
    else
      pthread_create(&ev_data.ev_t, NULL, &lirc_thread, &ev_data);
  }

  // Open connection to USB output device (Teensy3)
  if (usb[0] != 0)
  {
    fd_out = open(usb, O_RDWR | O_NONBLOCK | O_NDELAY);
    if (fd_out < 0)
      die("Error opening USB device %s: %s\n", usb, strerror(errno));

    /* *** Configure Port *** */
    struct termios tty;
    memset(&tty, 0, sizeof tty);

    /* Error Handling */
    if (tcgetattr(fd_out, &tty) != 0)
      die("Error getting tty attr: %s\n", strerror(errno));

    /* Set Baud Rate */
    cfsetospeed(&tty, B4000000);
    cfsetispeed(&tty, B4000000);

    /* Setting other Port Stuff */
    tty.c_cflag &= ~PARENB;        // Make 8n1
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;       // no flow control
    tty.c_lflag = 0;          // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                  // no remapping, no delays
    tty.c_cc[VMIN] = 0;                  // read doesn't block
    tty.c_cc[VTIME] = 5;                  // 0.5 seconds read timeout

    tty.c_cflag |= CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);// turn off s/w flow ctrl
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    tty.c_oflag &= ~OPOST;              // make raw

    /* Flush Port, then applies attributes */
    tcflush(fd_out, TCIFLUSH);

    if (tcsetattr(fd_out, TCSANOW, &tty) != 0)
      die("Error setting tty attr: %s\n", strerror(errno));

    printf("USB port: %s\n", usb);
  }
  else
  {
    fd_out = tcp_socket(host, port);
    if (fd_out < 0)
      die("Connect to %s:%d failed: %s\n", host, port, strerror(errno));

    printf("OPC host: %s:%d\n", host, port);
  }
  printf("Frame rate: %d fps, default: %d\n", ev_data.frame_rate, ev_data.base_frame_rate);
  printf("Brightness: %0.2f\n", ev_data.brightness_target);
  printf("Random start: %s\n", randomStart ? "yes" : "no");
  printf("Loop: %s\n", loop ? "yes" : "no");

  struct timeval t;
  gettimeofday(&t, NULL);
  const unsigned report_interval = 10;
  unsigned last_report = t.tv_sec;
  unsigned long delta_sum = 0;
  unsigned int frames = 0;
  unsigned int framesTotal = 0;
  ssize_t bytesTotal = 0;

  uint8_t buf[65536];
  int firstPacket = TRUE;

  srand(time(NULL));

  while (1)
  {
    opc_cmd_t cmd;
    ssize_t rlen = read(fd, &cmd, sizeof(cmd));
    bytesTotal += rlen;

    if (firstPacket)
    {
      int dataSize = cmd.len_hi << 8 | cmd.len_lo;
      int frameSize = dataSize + sizeof(cmd);

      if (randomStart)
      {
        randomStart = FALSE;
        // Choose a random spot max. 80% into the file
        float randomSpot = (rand() % 800) / 1000.0;
        int fSize = fileSize(fd);
        int fFrames = fSize / frameSize;
        int startFrame = (int)fFrames * randomSpot;
        lseek(fd, startFrame*frameSize, SEEK_SET);
        printf("%d frames in this file, starting at frame %d, position %d\n", fFrames, startFrame, startFrame*frameSize);
        continue; // read a new frame right away
      }

      firstPacket = FALSE;
      printf("OPC header: channel=%d command=%d size=%d\n", cmd.channel, cmd.command, dataSize);
    }

    if (rlen == 0)
    {
      printf("Bytes read: %u\n", bytesTotal);
      printf("Frames read: %u\n", framesTotal);
      framesTotal = 0;
      bytesTotal = 0;

      if (loop) {
        printf("Looping file\n");
        lseek(fd, 0, SEEK_SET); // seek to beginning of file
        rlen = read(fd, &cmd, sizeof(cmd)); // read cmd header again
        bytesTotal += rlen;
      }
      else {
        printf("Closing file\n");
        close(fd);
        fd = 0;
        break;
      }
    }

    // start timing
    struct timeval start_tv, stop_tv, delta_tv;
    gettimeofday(&start_tv, NULL);

    const size_t cmd_len = cmd.len_hi << 8 | cmd.len_lo;
    size_t offset = 0;

    while (offset < cmd_len)
    {
      rlen = read(fd, buf + offset, cmd_len - offset);
      if (rlen < 0)
        die("Read failed: %s\n", strerror(errno));
      if (rlen == 0)
        break;
      offset += rlen;

      bytesTotal += rlen;
    }

    if (cmd.command != 0)
      continue;

    // Calculate brightness
    if (ev_data.mode != 0)
      ev_data.brightness += (ev_data.brightness_target - ev_data.brightness) * .03;
    else
      ev_data.brightness -= ev_data.brightness * .02;

    //printf("%4.2f\n", ev_data.brightness);

    // Apply brightness to the OPC data
    for (int i = 0; i < cmd_len; i++)
      buf[i] = (int)((float)buf[i] * ev_data.brightness);

    // Send the data
    write(fd_out, &cmd, sizeof(cmd));
    write(fd_out, buf, sizeof(uint8_t)*cmd_len);

    // Write out any changed settings
    if (ev_data.write_delay > 0)
    {
      ev_data.write_delay--;
      if (ev_data.write_delay == 0)
      {
        int fd_cfg = open(cfg, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
        if (fd_cfg > 0)
        {
          printf("Writing settings to: %s\n", cfg);
          write(fd_cfg, &ev_data.brightness_target, sizeof(ev_data.brightness_target));
          write(fd_cfg, &ev_data.frame_rate, sizeof(ev_data.frame_rate));
          close(fd_cfg);
        }
      }
    }

    gettimeofday(&stop_tv, NULL);
    timersub(&stop_tv, &start_tv, &delta_tv);

    // wait for next frame
    int usec = 1000000 / ev_data.frame_rate - delta_tv.tv_usec - 180; // 180 is a magic number 
    if (usec > 0)
      usleep(usec);

    framesTotal++;
    frames++;
    delta_sum += delta_tv.tv_usec;

    if (stop_tv.tv_sec - last_report < report_interval)
      continue;

    last_report = stop_tv.tv_sec;

    const unsigned delta_avg = delta_sum / frames;
    printf("%u bytes, %u frames, %u usec avg, actual %.2f fps (over %u frames)\n",
      bytesTotal,
      framesTotal,
      delta_avg,
      frames * 1.0 / report_interval,
      frames
      );

    frames = delta_sum = 0;
  }

  return 0;
}
