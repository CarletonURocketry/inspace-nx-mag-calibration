#include <errno.h>
#include <float.h>
#include <math.h>
#include <nuttx/config.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <uORB/uORB.h>

ORB_DECLARE(sensor_mag);

struct sensor_mag_calib {
  float min_x;
  float max_x;
  float min_y;
  float max_y;
  float min_z;
  float max_z;
};

int main(int argc, char *argv[]) {
  struct orb_metadata const *mag_meta = ORB_ID(sensor_mag);
  if (mag_meta == NULL) {
    printf("Error getting uORB metadata: %s\n", strerror(errno));
    return -1;
  }

  int mag_fd = orb_subscribe_multi(mag_meta, 0);
  if (mag_fd < 0) {
    printf("Error subscribing to sensor_mag: %s\n", strerror(errno));
    return -1;
  }

  struct pollfd fds[] = {{.fd = mag_fd, .events = POLLIN, .revents = 0}};

  struct sensor_mag_calib mag_calib = {.min_x = FLT_MAX,
                                       .max_x = -FLT_MAX,
                                       .min_y = FLT_MAX,
                                       .max_y = -FLT_MAX,
                                       .min_z = FLT_MAX,
                                       .max_z = -FLT_MAX};

  int sample_count = 0;

  while (sample_count < CONFIG_INSPACE_MAG_CALIBRATION_N_SAMPLES) {
    int ret = poll(fds, 1, 1000);
    if (ret < 0) {
      printf("Poll error: %s\n", strerror(errno));
      continue;
    }

    if (ret == 0) {
      printf("Poll timeout, no data\n");
      continue;
    }

    if (fds[0].revents & POLLIN) {
      struct sensor_mag mag_data;
      ret = orb_copy(mag_meta, mag_fd, &mag_data);
      if (ret < 0) {
        printf("Error copying uORB data: %s\n", strerror(errno));
        continue;
      }

      mag_calib.min_x = fmin(mag_calib.min_x, mag_data.x);
      mag_calib.max_x = fmax(mag_calib.max_x, mag_data.x);
      mag_calib.min_y = fmin(mag_calib.min_y, mag_data.y);
      mag_calib.max_y = fmax(mag_calib.max_y, mag_data.y);
      mag_calib.min_z = fmin(mag_calib.min_z, mag_data.z);
      mag_calib.max_z = fmax(mag_calib.max_z, mag_data.z);

      sample_count++;
    }
  }

  float offset_x = (mag_calib.max_x + mag_calib.min_x) / 2.0f;
  float offset_y = (mag_calib.max_y + mag_calib.min_y) / 2.0f;
  float offset_z = (mag_calib.max_z + mag_calib.min_z) / 2.0f;

  printf("Hard iron offsets: X=%.2f, Y=%.2f, Z=%.2f uT\n", offset_x, offset_y,
         offset_z);

  close(mag_fd);
  return 0;
}
