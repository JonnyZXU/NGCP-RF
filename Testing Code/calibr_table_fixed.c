// servo_control.c
// Uses sysfs PWM: /sys/class/pwm/pwmchip0/pwm0
// Degree-based servo control with calibration-table interpolation.
//
// Instead of assuming one fixed linear "deg_per_us" scale across the
// whole rotation, this version uses measured calibration points:
//     degree <-> pulse width (us)
// and interpolates between the nearest points.
//
// This keeps the user interface degree-based while allowing nonlinear
// calibration.
//
// Build:
//   gcc servo_control.c -o servo_control -lm
//
// Before run (once per boot):
//   sudo pinctrl set 18 a3
//   echo 0 | sudo tee /sys/class/pwm/pwmchip0/export >/dev/null 2>&1 || true
//
// Run:
//   sudo ./servo_control
//
// Optional sensor stop (sysfs GPIO):
//   echo N | sudo tee /sys/class/gpio/export
//   echo in | sudo tee /sys/class/gpio/gpioN/direction
//   then:  sensor N

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <sys/stat.h>

#define PWM_CHIP "/sys/class/pwm/pwmchip0"
#define PWM_CH   "pwm0"
#define PWM_DIR  PWM_CHIP "/" PWM_CH
#define PWM_FILE(f) (PWM_DIR "/" f)

#define MAX_CAL_POINTS 32

static volatile sig_atomic_t stop_requested = 0;
static volatile sig_atomic_t exiting = 0;

static void sigint_handler(int s) {
    (void)s;
    stop_requested = 1;
    exiting = 1;
}

static int path_exists(const char *path) {
    struct stat st;
    return stat(path, &st) == 0;
}

static int wait_for_path(const char *path, int timeout_ms) {
    const int step = 10;
    for (int t = 0; t < timeout_ms; t += step) {
        if (path_exists(path)) return 0;
        usleep(step * 1000);
    }
    return -1;
}

static int write_str(const char *path, const char *val) {
    FILE *f = fopen(path, "w");
    if (!f) {
        fprintf(stderr, "ERROR open %s: %s\n", path, strerror(errno));
        return -1;
    }
    if (fputs(val, f) == EOF) {
        fprintf(stderr, "ERROR write %s: %s\n", path, strerror(errno));
        fclose(f);
        return -1;
    }
    fclose(f);
    return 0;
}

static int write_int(const char *path, long v) {
    char buf[64];
    snprintf(buf, sizeof(buf), "%ld", v);
    return write_str(path, buf);
}

static int read_int(const char *path, long *out) {
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    long v;
    int ok = fscanf(f, "%ld", &v);
    fclose(f);
    if (ok != 1) return -1;
    *out = v;
    return 0;
}

static int ensure_pwm(void) {
    if (!path_exists(PWM_DIR)) {
        if (write_str(PWM_CHIP "/export", "0") < 0) return -1;
        if (wait_for_path(PWM_DIR, 500) < 0) {
            fprintf(stderr, "ERROR: %s did not appear\n", PWM_DIR);
            return -1;
        }
    }

    write_str(PWM_FILE("enable"), "0");
    if (write_int(PWM_FILE("period"), 20000000) < 0) return -1;
    if (write_int(PWM_FILE("duty_cycle"), 1500000) < 0) return -1;
    if (write_str(PWM_FILE("enable"), "1") < 0) return -1;

    long en = 0;
    if (read_int(PWM_FILE("enable"), &en) < 0 || en != 1) {
        fprintf(stderr, "ERROR: PWM enable not 1 (got %ld)\n", en);
        return -1;
    }
    return 0;
}

static void pwm_off(void) {
    write_str(PWM_FILE("enable"), "0");
}

static int set_pwm_duty_ns(long duty_ns) {
    long period = 20000000;
    (void)read_int(PWM_FILE("period"), &period);
    if (duty_ns < 0) duty_ns = 0;
    if (duty_ns >= period) duty_ns = period - 1;
    return write_int(PWM_FILE("duty_cycle"), duty_ns);
}

static int read_gpio_value(int gpio, int *out) {
    char path[128];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    int c = fgetc(f);
    fclose(f);
    if (c == '0') { *out = 0; return 0; }
    if (c == '1') { *out = 1; return 0; }
    return -1;
}

static double clampd(double x, double lo, double hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

typedef struct {
    double deg;
    double pulse_us;
} cal_point_t;

typedef struct {
    double deg;
    double pulse_us;
} cal_point_t;

typedef enum {
    DIR_FWD = 1,
    DIR_REV = -1
} move_dir_t;

typedef struct {
    double center_us;
    double pulse_us;
    double deg_pos;
    double min_us;
    double max_us;

    cal_point_t cal_fwd[MAX_CAL_POINTS];
    int cal_fwd_count;

    cal_point_t cal_rev[MAX_CAL_POINTS];
    int cal_rev_count;
} servo_t;

static void sort_cal_table(cal_point_t *cal, int count) {
    for (int i = 0; i < count - 1; i++) {
        for (int j = i + 1; j < count; j++) {
            if (cal[j].deg < cal[i].deg) {
                cal_point_t tmp = cal[i];
                cal[i] = cal[j];
                cal[j] = tmp;
            }
        }
    }
}

static int validate_cal_table(const cal_point_t *cal, int count) {
    if (count < 2) return -1;
    for (int i = 0; i < count - 1; i++) {
        if (cal[i + 1].deg <= cal[i].deg) return -1;
    }
    return 0;
}

static int set_cal_point(cal_point_t *cal, int *count, double deg, double pulse_us) {
    for (int i = 0; i < *count; i++) {
        if (fabs(cal[i].deg - deg) < 1e-9) {
            cal[i].pulse_us = pulse_us;
            sort_cal_table(cal, *count);
            return validate_cal_table(cal, *count);
        }
    }

    if (*count >= MAX_CAL_POINTS) return -1;

    cal[*count].deg = deg;
    cal[*count].pulse_us = pulse_us;
    (*count)++;

    sort_cal_table(cal, *count);
    return validate_cal_table(cal, *count);
}

static void print_cal_table(const char *name, const cal_point_t *cal, int count) {
    printf("%s calibration table:\n", name);
    for (int i = 0; i < count; i++) {
        printf("  %2d: deg=%8.3f  pulse_us=%8.3f\n",
               i, cal[i].deg, cal[i].pulse_us);
    }
}

static const cal_point_t *get_cal_table_const(const servo_t *s, move_dir_t dir, int *count) {
    if (dir == DIR_REV) {
        *count = s->cal_rev_count;
        return s->cal_rev;
    } else {
        *count = s->cal_fwd_count;
        return s->cal_fwd;
    }
}

static cal_point_t *get_cal_table(servo_t *s, move_dir_t dir, int *count) {
    if (dir == DIR_REV) {
        *count = s->cal_rev_count;
        return s->cal_rev;
    } else {
        *count = s->cal_fwd_count;
        return s->cal_fwd;
    }
}

static double servo_deg_to_pulse_us_dir(const servo_t *s, double deg, move_dir_t dir) {
    int count = 0;
    const cal_point_t *cal = get_cal_table_const(s, dir, &count);

    if (count < 2) return s->center_us;

    if (deg <= cal[0].deg) return cal[0].pulse_us;
    if (deg >= cal[count - 1].deg) return cal[count - 1].pulse_us;

    for (int i = 0; i < count - 1; i++) {
        double d0 = cal[i].deg;
        double d1 = cal[i + 1].deg;

        if (deg >= d0 && deg <= d1) {
            double frac = (deg - d0) / (d1 - d0);
            return cal[i].pulse_us + frac * (cal[i + 1].pulse_us - cal[i].pulse_us);
        }
    }

    return cal[count - 1].pulse_us;
}

static double servo_pulse_to_deg(const servo_t *s, double pulse_us) {
    const cal_point_t *cal = s->cal_fwd;
    int count = s->cal_fwd_count;

    if (count < 2) return 0.0;

    int increasing = (cal[count - 1].pulse_us >= cal[0].pulse_us);

    if (increasing) {
        if (pulse_us <= cal[0].pulse_us) return cal[0].deg;
        if (pulse_us >= cal[count - 1].pulse_us) return cal[count - 1].deg;

        for (int i = 0; i < count - 1; i++) {
            double p0 = cal[i].pulse_us;
            double p1 = cal[i + 1].pulse_us;
            if (pulse_us >= p0 && pulse_us <= p1) {
                double frac = (pulse_us - p0) / (p1 - p0);
                return cal[i].deg + frac * (cal[i + 1].deg - cal[i].deg);
            }
        }
    } else {
        if (pulse_us >= cal[0].pulse_us) return cal[0].deg;
        if (pulse_us <= cal[count - 1].pulse_us) return cal[count - 1].deg;

        for (int i = 0; i < count - 1; i++) {
            double p0 = cal[i].pulse_us;
            double p1 = cal[i + 1].pulse_us;
            if (pulse_us <= p0 && pulse_us >= p1) {
                double frac = (pulse_us - p0) / (p1 - p0);
                return cal[i].deg + frac * (cal[i + 1].deg - cal[i].deg);
            }
        }
    }

    return cal[count - 1].deg;
}

static void servo_recalc_deg_from_pulse(servo_t *s) {
    s->deg_pos = servo_pulse_to_deg(s, s->pulse_us);
}

static int servo_set_pulse_us(servo_t *s, double pulse_us) {
    pulse_us = clampd(pulse_us, s->min_us, s->max_us);
    s->pulse_us = pulse_us;
    servo_recalc_deg_from_pulse(s);
    long duty_ns = (long) llround(pulse_us * 1000.0);
    return set_pwm_duty_ns(duty_ns);
}

static int servo_ramp_to_pulse(servo_t *s, double target_us, double duration_s,
                               int sensor_gpio, int dt_ms)
{
    target_us = clampd(target_us, s->min_us, s->max_us);
    if (duration_s <= 0.0) duration_s = 0.05;
    if (dt_ms < 10) dt_ms = 10;

    int steps = (int)((duration_s * 1000.0) / dt_ms);
    if (steps < 1) steps = 1;

    double start = s->pulse_us;
    for (int i = 0; i <= steps; i++) {
        if (stop_requested) return -1;

        double frac = (double)i / (double)steps;
        double pu = start + frac * (target_us - start);

        if (servo_set_pulse_us(s, pu) < 0) return -2;

        if (sensor_gpio >= 0) {
            int v;
            if (read_gpio_value(sensor_gpio, &v) == 0 && v) return 1;
        }

        usleep(dt_ms * 1000);
    }
    return 0;
}

static int servo_hold(double hold_s, int sensor_gpio) {
    if (hold_s <= 0.0) return 0;
    const int poll_ms = 50;
    int loops = (int)((hold_s * 1000.0) / poll_ms);
    if (loops < 1) loops = 1;

    for (int i = 0; i < loops; i++) {
        if (stop_requested) return -1;
        if (sensor_gpio >= 0) {
            int v;
            if (read_gpio_value(sensor_gpio, &v) == 0 && v) return 1;
        }
        usleep(poll_ms * 1000);
    }
    return 0;
}

static void help(void) {
    puts("Commands:");
    puts("  status");
    puts("  cal show");
    puts("  cal set <fwd|rev> <deg> <pulse_us>    (add/update calibration point)");
    puts("  pulse <us>                  (direct pulse command)");
    puts("  move <deg> <sec> [hold_s]   (relative move in degrees)");
    puts("  goto <deg> <sec> [hold_s]   (absolute degree using interpolation)");
    puts("  range <a> <b> <sec> [hold]  (sweep a->b then optional hold)");
    puts("  search <a> <b> <sec>        (sweep and stop if sensor triggers)");
    puts("  auto <mode> [rot] [sec/rot] [hold_s]");
    puts("      mode 0=5deg, 1=15deg, 2=30deg, 3=45deg");
    puts("      stops at each milestone and holds");
    puts("  sensor <gpioN>              (sysfs gpio input; stops motion when reads 1)");
    puts("  stop                        (immediate stop of motion)");
    puts("  quit");
}

int main(void) {
    signal(SIGINT, sigint_handler);

    if (ensure_pwm() < 0) {
        fprintf(stderr, "PWM init failed. Make sure pwm0 exists and GPIO18 is set to PWM (sudo pinctrl set 18 a3).\n");
        return 1;
    }

    servo_t s = {
        .center_us = 1500.0,
        .pulse_us  = 1500.0,
        .deg_pos   = 0.0,
        .min_us    = 500.0,
        .max_us    = 2500.0,
        .cal_fwd_count = 17,
        .cal_rev_count = 17
    };

      s.cal_fwd[0] = (cal_point_t){   0.0, 1499.0 };
      s.cal_fwd[1] = (cal_point_t){  45.0, 1546.0 };
      s.cal_fwd[2] = (cal_point_t){  90.0, 1601.0 };
      s.cal_fwd[3] = (cal_point_t){ 135.0, 1653.0 };
      s.cal_fwd[4] = (cal_point_t){ 180.0, 1702.75 };
      s.cal_fwd[5] = (cal_point_t){ 225.0, 1754.5 };
      s.cal_fwd[6] = (cal_point_t){ 270.0, 1807.0 };
      s.cal_fwd[7] = (cal_point_t){ 315.0, 1856.5 };
      s.cal_fwd[8] = (cal_point_t){ 360.0, 1906.5 };
      s.cal_fwd[9] = (cal_point_t){ 405.0, 1953.75 };
      s.cal_fwd[10] = (cal_point_t){ 450.0, 2001.0 };
      s.cal_fwd[11] = (cal_point_t){ 495.0, 2055.5 };
      s.cal_fwd[12] = (cal_point_t){ 540.0, 2104.0 };
      s.cal_fwd[13] = (cal_point_t){ 585.0, 2150.0 };
      s.cal_fwd[14] = (cal_point_t){ 630.0, 2197.0 };
      s.cal_fwd[15] = (cal_point_t){ 675.0, 2246.0 };
      s.cal_fwd[16] = (cal_point_t){ 720.0, 2296.0 };
      
      s.cal_rev[0] = (cal_point_t){   0.0, 1499.0 };
      s.cal_rev[1] = (cal_point_t){  45.0, 1546.0 };
      s.cal_rev[2] = (cal_point_t){  90.0, 1601.0 };
      s.cal_rev[3] = (cal_point_t){ 135.0, 1653.0 };
      s.cal_rev[4] = (cal_point_t){ 180.0, 1702.75 };
      s.cal_rev[5] = (cal_point_t){ 225.0, 1754.5 };
      s.cal_rev[6] = (cal_point_t){ 270.0, 1807.0 };
      s.cal_rev[7] = (cal_point_t){ 315.0, 1856.5 };
      s.cal_rev[8] = (cal_point_t){ 360.0, 1906.5 };
      s.cal_rev[9] = (cal_point_t){ 405.0, 1953.75 };
      s.cal_rev[10] = (cal_point_t){ 450.0, 2001.0 };
      s.cal_rev[11] = (cal_point_t){ 495.0, 2055.5 };
      s.cal_rev[12] = (cal_point_t){ 540.0, 2104.0 };
      s.cal_rev[13] = (cal_point_t){ 585.0, 2150.0 };
      s.cal_rev[14] = (cal_point_t){ 630.0, 2197.0 };
      s.cal_rev[15] = (cal_point_t){ 675.0, 2246.0 };
      s.cal_rev[16] = (cal_point_t){ 720.0, 2296.0 };

    sort_cal_table(s.cal_fwd, s.cal_fwd_count);
    sort_cal_table(s.cal_rev, s.cal_rev_count);
    
    if (validate_cal_table(s.cal_fwd, s.cal_fwd_count) < 0) {
        fprintf(stderr, "Invalid forward calibration table.\n");
        return 1;
    }
    if (validate_cal_table(s.cal_rev, s.cal_rev_count) < 0) {
        fprintf(stderr, "Invalid reverse calibration table.\n");
        return 1;
    }

    if (servo_set_pulse_us(&s, servo_deg_to_pulse_us_dir(&s, 0.0, DIR_FWD)) < 0) {
        fprintf(stderr, "Failed to set initial pulse.\n");
        return 1;
    }
    usleep(200000);

    int sensor_gpio = -1;

    puts("Ready. Type 'help' for commands.");
    char line[256];

    while (!exiting) {
        printf("> ");
        fflush(stdout);
        if (!fgets(line, sizeof(line), stdin)) break;

        char *p = line;
        while (*p && isspace((unsigned char)*p)) p++;
        if (*p == '\0') continue;

        char cmd[32];
        if (sscanf(p, "%31s", cmd) != 1) continue;

        if (!strcmp(cmd, "help")) { help(); continue; }
        if (!strcmp(cmd, "quit")) break;

        if (!strcmp(cmd, "stop")) {
            stop_requested = 1;
            puts("Stop requested.");
            continue;
        }

        if (!strcmp(cmd, "status")) {
            printf("pulse_us=%.2f  deg_pos(model)=%.2f  center_us=%.1f  clamp=[%.0f..%.0f]  sensor=%d  cal_points=%d\n",
                   s.pulse_us, s.deg_pos, s.center_us, s.min_us, s.max_us, sensor_gpio, s.cal_count);
            continue;
        }

        if (!strcmp(cmd, "cal")) {
            char sub[32];
            if (sscanf(p + 3, "%31s", sub) == 1) {
                if (!strcmp(sub, "show")) {
                    print_cal_table("Forward", s.cal_fwd, s.cal_fwd_count);
                    print_cal_table("Reverse", s.cal_rev, s.cal_rev_count);
                } else if (!strcmp(sub, "set")) {
                    char which[32];
                    double deg, pulse_us;
                    if (sscanf(p + 3, "%*s %31s %lf %lf", which, &deg, &pulse_us) == 3) {
                        if (pulse_us < s.min_us || pulse_us > s.max_us) {
                            printf("Pulse %.2f is outside clamp range [%.0f..%.0f]\n",
                                   pulse_us, s.min_us, s.max_us);
                        } else {
                            int rc = -1;
        
                            if (!strcmp(which, "fwd")) {
                                rc = set_cal_point(s.cal_fwd, &s.cal_fwd_count, deg, pulse_us);
                            } else if (!strcmp(which, "rev")) {
                                rc = set_cal_point(s.cal_rev, &s.cal_rev_count, deg, pulse_us);
                            } else {
                                puts("Usage: cal set <fwd|rev> <deg> <pulse_us>");
                                continue;
                            }
        
                            if (rc < 0) {
                                puts("Failed to add/update calibration point.");
                            } else {
                                servo_recalc_deg_from_pulse(&s);
                                printf("Calibration point set: table=%s deg=%.3f pulse_us=%.3f\n",
                                       which, deg, pulse_us);
                            }
                        }
                    } else {
                        puts("Usage: cal set <fwd|rev> <deg> <pulse_us>");
                    }
                } else {
                    puts("Usage: cal show | cal set <fwd|rev> <deg> <pulse_us>");
                }
            } else {
                puts("Usage: cal show | cal set <fwd|rev> <deg> <pulse_us>");
            }
            continue;
        }

        if (!strcmp(cmd, "sensor")) {
            int g;
            if (sscanf(p + 6, "%d", &g) == 1) {
                sensor_gpio = g;
                printf("Sensor set to gpio%d (sysfs). It must be exported and set as input.\n", g);
            } else {
                puts("Usage: sensor <gpioN>");
            }
            continue;
        }

        if (!strcmp(cmd, "pulse")) {
            double us;
            if (sscanf(p + 5, "%lf", &us) == 1) {
                stop_requested = 0;
                if (servo_set_pulse_us(&s, us) < 0) puts("Failed to set duty.");
                else printf("Set pulse %.2f us (deg_pos model %.2f)\n", s.pulse_us, s.deg_pos);
            } else {
                puts("Usage: pulse <us>");
            }
            continue;
        }

        if (!strcmp(cmd, "move")) {
            double deg, sec, hold_s = 0.0;
            int n = sscanf(p + 4, "%lf %lf %lf", &deg, &sec, &hold_s);
            if (n >= 2) {
                stop_requested = 0;
        
                double target_deg = s.deg_pos + deg;
                move_dir_t dir = (deg < 0.0) ? DIR_REV : DIR_FWD;
                double target = servo_deg_to_pulse_us_dir(&s, target_deg, dir);
        
                printf("Move %.2f deg from %.2f -> target %.2f deg using %s table -> pulse %.2f us\n",
                       deg, s.deg_pos, target_deg,
                       (dir == DIR_REV) ? "reverse" : "forward",
                       target);
        
                int rc = servo_ramp_to_pulse(&s, target, sec, sensor_gpio, 20);
                if (rc == 1) puts("Stopped by sensor.");
                else if (rc < 0) puts("Stopped.");
                else {
                    int hc = servo_hold(hold_s, sensor_gpio);
                    if (hc == 1) puts("Stopped by sensor during hold.");
                }
            } else {
                puts("Usage: move <deg> <sec> [hold_s]");
            }
            continue;
        }

        if (!strcmp(cmd, "goto")) {
            double deg, sec, hold_s = 0.0;
            int n = sscanf(p + 4, "%lf %lf %lf", &deg, &sec, &hold_s);
            if (n >= 2) {
                stop_requested = 0;
        
                move_dir_t dir = (deg < s.deg_pos) ? DIR_REV : DIR_FWD;
                double target = servo_deg_to_pulse_us_dir(&s, deg, dir);
        
                printf("Goto %.2f deg using %s table -> target pulse %.2f us\n",
                       deg, (dir == DIR_REV) ? "reverse" : "forward", target);
        
                int rc = servo_ramp_to_pulse(&s, target, sec, sensor_gpio, 20);
                if (rc == 1) puts("Stopped by sensor.");
                else if (rc < 0) puts("Stopped.");
                else {
                    int hc = servo_hold(hold_s, sensor_gpio);
                    if (hc == 1) puts("Stopped by sensor during hold.");
                }
            } else {
                puts("Usage: goto <deg> <sec> [hold_s]");
            }
            continue;
        }

        if (!strcmp(cmd, "range") || !strcmp(cmd, "search")) {
            double a, b, sec, hold_s = 0.0;
            int is_search = (!strcmp(cmd, "search"));

            int n = is_search
                ? sscanf(p + 6, "%lf %lf %lf", &a, &b, &sec)
                : sscanf(p + 5, "%lf %lf %lf %lf", &a, &b, &sec, &hold_s);

            if ((is_search && n == 3) || (!is_search && n >= 3)) {
                stop_requested = 0;
                move_dir_t dir_to_a = (a < s.deg_pos) ? DIR_REV : DIR_FWD;
                move_dir_t dir_a_to_b = (b < a) ? DIR_REV : DIR_FWD;
                
                double pa = servo_deg_to_pulse_us_dir(&s, a, dir_to_a);
                double pb = servo_deg_to_pulse_us_dir(&s, b, dir_a_to_b);

                printf("%s %.2f->%.2f deg (pulse %.2f->%.2f us) over %.2fs\n",
                       is_search ? "Search" : "Range", a, b, pa, pb, sec);

                int rc = servo_ramp_to_pulse(&s, pa, 0.2, sensor_gpio, 20);
                if (rc == 1) { puts("Sensor triggered (while moving to start)."); continue; }
                if (rc < 0) { puts("Stopped."); continue; }

                rc = servo_ramp_to_pulse(&s, pb, sec, sensor_gpio, 20);
                if (rc == 1) puts("Sensor triggered (stopped).");
                else if (rc < 0) puts("Stopped.");
                else if (!is_search) {
                    int hc = servo_hold(hold_s, sensor_gpio);
                    if (hc == 1) puts("Sensor triggered during hold.");
                }
            } else {
                if (is_search) puts("Usage: search <start_deg> <end_deg> <sec>");
                else puts("Usage: range <start_deg> <end_deg> <sec> [hold_s]");
            }
            continue;
        }

        if (!strcmp(cmd, "auto")) {
            int mode = -1;
            double rot = 1.0, sec_per_rot = 60.0, hold_s = 0.5;
            int n = sscanf(p + 4, "%d %lf %lf %lf", &mode, &rot, &sec_per_rot, &hold_s);
            if (n < 1) { puts("Usage: auto <mode 0..3> [rot] [sec/rot] [hold_s]"); continue; }
            if (mode < 0 || mode > 3) { puts("Mode must be 0..3"); continue; }
            if (rot <= 0.0) rot = 1.0;
            if (sec_per_rot <= 0.0) sec_per_rot = 60.0;
            if (hold_s < 0.0) hold_s = 0.0;

            int step = (mode == 0) ? 5 : (mode == 1) ? 15 : (mode == 2) ? 30 : 45;
            double total_deg = 360.0 * rot;
            int num_steps = (int)llround(total_deg / step);
            if (num_steps < 1) num_steps = 1;

            double total_time = rot * sec_per_rot;
            double per_step = total_time / num_steps;
            double move_time = per_step - hold_s;
            if (move_time < 0.05) move_time = 0.05;

            printf("Auto mode %d: step=%ddeg rot=%.2f sec/rot=%.2f move/step=%.2fs hold=%.2fs\n",
                   mode, step, rot, sec_per_rot, move_time, hold_s);

            stop_requested = 0;

            int rc = servo_ramp_to_pulse(&s, servo_deg_to_pulse_us_dir(&s, 0.0, DIR_FWD), 0.5, sensor_gpio, 20);
            if (rc == 1) { puts("Sensor triggered."); continue; }
            if (rc < 0) { puts("Stopped."); continue; }

            for (int k = 0; k <= num_steps && !stop_requested; k++) {
                double target_deg = k * (double)step;
                double shown = target_deg;

                move_dir_t dir = (shown < s.deg_pos) ? DIR_REV : DIR_FWD;
                double target_pulse = servo_deg_to_pulse_us_dir(&s, shown, dir);

                printf("  stop at %.0f deg -> pulse %.2f us\n",
                       shown, clampd(target_pulse, s.min_us, s.max_us));

                rc = servo_ramp_to_pulse(&s, target_pulse, move_time, sensor_gpio, 20);
                if (rc == 1) { printf("Sensor triggered at %.0f deg\n", shown); break; }
                if (rc < 0) { puts("Stopped."); break; }

                rc = servo_hold(hold_s, sensor_gpio);
                if (rc == 1) { printf("Sensor triggered during hold at %.0f deg\n", shown); break; }
                if (rc < 0) { puts("Stopped."); break; }
            }

            puts("Auto finished or stopped.");
            continue;
        }

        puts("Unknown command. Type 'help'.");
    }

    puts("Exiting. Disabling PWM.");
    pwm_off();
    return 0;
