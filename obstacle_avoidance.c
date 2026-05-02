#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/utils/motion.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#define TIME_STEP 64

// ───── Obstacle detection ─────
#define OBSTACLE_THRESHOLD 120   // darker pixels (floor/objects)
#define SCAN_ROWS 15
#define MIN_CLUSTER_SIZE 40

// ───── Steering ─────
#define CENTER_TOLERANCE 40

static WbDeviceTag cam_top;

static int width_top, height_top;

static WbMotionRef walk_forward, step_left, step_right, wave;

// ─────────────────────────────
// detect dark obstacle clusters
int detect_obstacle(const unsigned char *image, int width, int height,
                    int *out_size) {

  int best_size = 0;
  int best_x = -1;

  for (int row = 0; row < SCAN_ROWS; row++) {

    int y = height - 1 - row * 3;

    int cluster_start = -1;
    int cluster_size = 0;

    for (int x = 0; x < width; x++) {

      int r = wb_camera_image_get_red(image, width, x, y);
      int g = wb_camera_image_get_green(image, width, x, y);
      int b = wb_camera_image_get_blue(image, width, x, y);

      // obstacle = darker than threshold
      if (r < OBSTACLE_THRESHOLD &&
          g < OBSTACLE_THRESHOLD &&
          b < OBSTACLE_THRESHOLD) {

        if (cluster_start == -1)
          cluster_start = x;

        cluster_size++;

      } else {
        if (cluster_start != -1 && cluster_size > best_size) {
          best_size = cluster_size;
          best_x = cluster_start + cluster_size / 2;
        }

        cluster_start = -1;
        cluster_size = 0;
      }
    }

    if (cluster_start != -1 && cluster_size > best_size) {
      best_size = cluster_size;
      best_x = cluster_start + cluster_size / 2;
    }
  }

  if (out_size)
    *out_size = best_size;

  return best_x;
}

// ─────────────────────────────
void play(WbMotionRef motion, const char *label) {
  printf("▶️ %s\n", label);
  wbu_motion_play(motion);

  while (!wbu_motion_is_over(motion))
    wb_robot_step(TIME_STEP);
}

// ─────────────────────────────
void init() {

  cam_top = wb_robot_get_device("CameraTop");
  wb_camera_enable(cam_top, TIME_STEP);

  width_top = wb_camera_get_width(cam_top);
  height_top = wb_camera_get_height(cam_top);

  walk_forward = wbu_motion_new("../../motions/Forwards50.motion");
  step_left    = wbu_motion_new("../../motions/SideStepLeft.motion");
  step_right   = wbu_motion_new("../../motions/SideStepRight.motion");
  wave         = wbu_motion_new("../../motions/HandWave.motion");

  if (!walk_forward || !step_left || !step_right) {
    printf("Motion load failed\n");
    exit(1);
  }

  printf("🤖 Obstacle Avoidance Started\n");
}

// ─────────────────────────────
int main() {

  wb_robot_init();
  init();

  while (wb_robot_step(TIME_STEP) != -1) {

    const unsigned char *img = wb_camera_get_image(cam_top);
    if (!img) continue;

    int cluster_size = 0;
    int obs_x = detect_obstacle(img, width_top, height_top, &cluster_size);

    int center = width_top / 2;

    bool obstacle_detected = (obs_x != -1 && cluster_size > MIN_CLUSTER_SIZE);

    printf("Obstacle x=%d size=%d\n", obs_x, cluster_size);

    // ───── Decision ─────
    if (!obstacle_detected) {

      play(walk_forward, "Forward");

    } else {

      // obstacle in left or right
      if (obs_x < center) {
        play(step_right, "Avoid RIGHT");
      } else {
        play(step_left, "Avoid LEFT");
      }
    }
  }

  wb_robot_cleanup();
  return 0;
}