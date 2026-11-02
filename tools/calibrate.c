// Voice-cued, continuous capture of Maxwell 2 orientation quaternions (HID report 0x0A)
// for axis calibration. Cues are spoken with `say`, so you don't need to watch the screen.
// Usage: calibrate <hidapi-providing dylib> <out.csv>
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#include <sys/time.h>

struct hid_device_info { char *path; unsigned short vendor_id, product_id; wchar_t *serial_number; unsigned short release_number;
  wchar_t *manufacturer_string, *product_string; unsigned short usage_page, usage; int interface_number; struct hid_device_info *next; };

static double now(void) { struct timeval tv; gettimeofday(&tv, 0); return tv.tv_sec + tv.tv_usec / 1e6; }

int main(int argc, char **argv) {
  void *l = dlopen(argv[1], RTLD_NOW);
  struct hid_device_info *(*enumerate)(unsigned short, unsigned short) = dlsym(l, "hid_enumerate");
  void *(*open_path)(const char *) = dlsym(l, "hid_open_path");
  int (*hwrite)(void *, const unsigned char *, size_t) = dlsym(l, "hid_write");
  int (*hread_to)(void *, unsigned char *, size_t, int) = dlsym(l, "hid_read_time_out");
  ((int (*)(void))dlsym(l, "hid_init"))();

  void *d = 0;
  for (struct hid_device_info *i = enumerate(0x3329, 0x4baa); i; i = i->next)
    if (i->usage_page == 0xff13 && i->usage == 1) d = open_path(i->path);
  if (!d) { printf("Maxwell 2 not found\n"); return 1; }

  unsigned char en[62] = {0x06, 0x09, 0x00, 0x05, 0x5a, 0x05, 0x00, 0x00, 0x09, 0x2e, 0x00, 0xbc};
  hwrite(d, en, sizeof en);

  // Cue times (seconds from start) and what gets spoken.
  struct { double at; const char *phase, *say; } cues[] = {
    { 0, "rest0", "Calibration starting. Face straight ahead and hold still."},
    { 6, "yaw",   "Turn your head to the left, and hold."},
    {11, "rest1", "Back to center."},
    {16, "pitch", "Look up at the ceiling, and hold."},
    {21, "rest2", "Back to center."},
    {26, "roll",  "Tilt your right ear down to your right shoulder, and hold."},
    {32, "rest3", "Back to center."},
    {37, "end",   "Done. Thank you."},
  };
  const int ncues = sizeof cues / sizeof *cues;

  FILE *out = fopen(argv[2], "w");
  fprintf(out, "t,phase,a,b,c,d\n");
  printf("\nMaxwell 2 calibration (about 40 s). Put the headset on and follow the spoken cues.\n"
         "You don't need to look at the screen.\n\nPress Enter to start... ");
  fflush(stdout);
  for (int c; (c = getchar()) != '\n' && c != EOF;) {}
  { unsigned char b[64]; while (hread_to(d, b, sizeof b, 0) > 0) {} }
  double t0 = now();
  int next = 0;
  const char *phase = "pre";
  while (next < ncues || now() - t0 < cues[ncues - 1].at + 1) {
    double t = now() - t0;
    if (next < ncues && t >= cues[next].at) {
      char cmd[256];
      snprintf(cmd, sizeof cmd, "say '%s' &", cues[next].say);
      system(cmd);
      phase = cues[next].phase;
      fprintf(out, "%.3f,cue_%s,0,0,0,0\n", t, phase);
      printf("[%5.1fs] %s\n", t, cues[next].say); fflush(stdout);
      next++;
    }
    unsigned char b[64]; int n = hread_to(d, b, sizeof b, 10);
    if (n >= 17 && b[0] == 0x0a) {
      float q[4]; memcpy(q, b + 1, 16);
      fprintf(out, "%.3f,%s,%f,%f,%f,%f\n", now() - t0, phase, q[0], q[1], q[2], q[3]);
    }
  }
  en[11] = 0x3c; hwrite(d, en, sizeof en);
  fclose(out);
  printf("\nDone - you can close this tab.\n");
  return 0;
}
