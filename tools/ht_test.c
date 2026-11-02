// Loads a head-tracker plugin the way the Dolby Atmos Renderer does and prints its orientation.
// Usage: ht_test <plugin.dylib> [seconds] [live]
// Angles use the Renderer's convention: yaw + = turned left, pitch + = looking up, roll + = right ear down.
#include <dlfcn.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

typedef struct { const char *name, *developer; } desc_t;

int main(int argc, char **argv) {
  double secs = argc > 2 ? atof(argv[2]) : 3;
  int live = argc > 3 && !strcmp(argv[3], "live");
  void *lib = dlopen(argv[1], RTLD_NOW);
  if (!lib) { printf("dlopen failed: %s\n", dlerror()); return 1; }
  void (*describe)(desc_t *) = dlsym(lib, "headtracker_describe");
  void *(*create)(void) = dlsym(lib, "headtracker_create");
  void (*run)(void *) = dlsym(lib, "headtracker_run");
  int (*rot)(void *, float *) = dlsym(lib, "headtracker_get_rotation_matrix");
  void (*zero)(void *) = dlsym(lib, "headtracker_set_zero_reference");
  int (*err)(void *) = dlsym(lib, "headtracker_get_error");
  const char *(*errstr)(void *, int) = dlsym(lib, "headtracker_get_error_string");
  void (*cancel)(void *) = dlsym(lib, "headtracker_cancel");
  void (*destroy)(void *) = dlsym(lib, "headtracker_destroy");
  if (!describe || !create || !run || !rot || !zero || !err || !errstr || !cancel || !destroy) { printf("missing symbol\n"); return 1; }

  desc_t d; describe(&d);
  printf("plugin: %s / %s\n", d.name, d.developer);
  void *h = create();
  printf("create -> %s\n", h ? "ok" : "NULL");
  if (!h) return 2;
  // The Renderer calls run on its own tracking thread; run blocks until cancel.
  pthread_t runner;
  pthread_create(&runner, NULL, (void *(*)(void *))run, h);
  int steps = (int)(secs * 10), oks = 0;
  float ymin = 1e9, ymax = -1e9;
  for (int i = 0; i < steps; i++) {
    float m[9]; int ok = rot(h, m);
    float yaw = atan2f(m[3], m[0]) * 180 / M_PI, pitch = asinf(fmaxf(-1, fminf(1, m[6]))) * 180 / M_PI,
          roll = atan2f(m[7], m[8]) * 180 / M_PI;
    float det = m[0]*(m[4]*m[8]-m[5]*m[7]) - m[1]*(m[3]*m[8]-m[5]*m[6]) + m[2]*(m[3]*m[7]-m[4]*m[6]);
    if (ok) { oks++; ymin = fminf(ymin, yaw); ymax = fmaxf(ymax, yaw); }
    if (live) {
      printf("\r%s  yaw %+6.1f (left +)   pitch %+6.1f (up +)   roll %+6.1f (right ear down +)   ",
             ok ? "TRACKING" : errstr(h, err(h)), yaw, pitch, roll);
      fflush(stdout);
    } else if (i % 5 == 0) {
      printf("t=%.1fs ok=%d err=%d(%s) yaw=%+6.2f pitch=%+6.2f roll=%+6.2f det=%.4f\n",
             i / 10.0, ok, err(h), errstr(h, err(h)), yaw, pitch, roll, det);
    }
    if (i == steps / 2) zero(h);
    usleep(100000);
  }
  printf("\nvalid samples %d/%d, yaw range %.2f..%.2f\n", oks, steps, ymin, ymax);
  cancel(h);
  pthread_join(runner, NULL);
  destroy(h);
  printf("cancel/destroy ok\n");
  return 0;
}
