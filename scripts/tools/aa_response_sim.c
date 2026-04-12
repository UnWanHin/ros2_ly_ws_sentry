// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.
//
// aa_response_sim.c
// SDL2-based response simulator for target -> predictor -> BT -> gimbal chain.
//
// Build:
//   gcc -O2 -std=c11 -Wall -Wextra -o /tmp/aa_response_sim
//   scripts/tools/aa_response_sim.c $(pkg-config --cflags --libs sdl2) -lm
//
// Run:
//   /tmp/aa_response_sim --mode step --step-interval-sec 0.7 --obs-alpha 0.25

#include <SDL2/SDL.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_SAMPLES 240000
#define MAX_EVENTS 2048
#define PI_D 3.14159265358979323846

typedef enum {
    MODE_STEP = 0,
    MODE_SINE = 1,
    MODE_ZIGZAG = 2
} TargetMode;

typedef enum {
    NO_TARGET_HOLD = 0,
    NO_TARGET_SCAN = 1
} NoTargetMode;

typedef struct {
    double t;
    double target_yaw;
    double predictor_yaw;
    double gimbal_yaw;
} Sample;

typedef struct {
    double start_t;
    double target_yaw;
    double in_band_since;
    bool settled;
    double settle_time_sec;
} StepEvent;

typedef struct {
    double sim_hz;
    double tracker_hz;
    double publish_hz;
    double duration_sec;

    TargetMode mode;
    double amp_deg;
    double period_sec;
    double step_interval_sec;

    double obs_alpha;
    double meas_noise_deg;
    double dropout_prob;

    int publish_only_on_new_frame;
    int require_fresh_observation;
    double coast_timeout_sec;

    int bt_reuse_latched;
    double lost_target_hold_ms;
    NoTargetMode no_target_mode;
    double scan_speed_deg_s;
    double scan_range_deg;

    double gimbal_max_rate_deg_s;
    double history_sec;
    int headless;
} SimConfig;

typedef struct {
    Sample samples[MAX_SAMPLES];
    int write_idx;
    int count;

    StepEvent events[MAX_EVENTS];
    int event_count;

    double mae_sum;
    double max_abs_err;
    int err_count;
    double *errors;

    double predictor_yaw;
    bool predictor_initialized;
    double gimbal_yaw;
    double bt_cmd_yaw;
    double latched_yaw;
    bool has_latched;
    int scan_dir;

    double last_obs_t;
    double last_target_seen_t;
    bool has_new_tracker_frame;

    double tracker_accum;
    double publish_accum;
    double sim_t;

    bool paused;
    bool quit;
} SimState;

static double clampd(const double x, const double lo, const double hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

static double wrap_deg(double a) {
    while (a > 180.0) a -= 360.0;
    while (a < -180.0) a += 360.0;
    return a;
}

static double angle_diff_deg(const double target, const double current) {
    return wrap_deg(target - current);
}

static double frand01(void) {
    return (double)rand() / (double)RAND_MAX;
}

static double uniform_noise(const double amp) {
    return (frand01() * 2.0 - 1.0) * amp;
}

static TargetMode parse_mode(const char *s) {
    if (strcmp(s, "step") == 0) return MODE_STEP;
    if (strcmp(s, "sine") == 0) return MODE_SINE;
    if (strcmp(s, "zigzag") == 0) return MODE_ZIGZAG;
    return MODE_STEP;
}

static NoTargetMode parse_no_target_mode(const char *s) {
    if (strcmp(s, "scan") == 0) return NO_TARGET_SCAN;
    return NO_TARGET_HOLD;
}

static void set_default_config(SimConfig *cfg) {
    cfg->sim_hz = 200.0;
    cfg->tracker_hz = 60.0;
    cfg->publish_hz = 100.0;
    cfg->duration_sec = 30.0;

    cfg->mode = MODE_STEP;
    cfg->amp_deg = 45.0;
    cfg->period_sec = 1.2;
    cfg->step_interval_sec = 0.8;

    cfg->obs_alpha = 0.25;
    cfg->meas_noise_deg = 0.5;
    cfg->dropout_prob = 0.0;

    cfg->publish_only_on_new_frame = 1;
    cfg->require_fresh_observation = 1;
    cfg->coast_timeout_sec = 0.5;

    cfg->bt_reuse_latched = 1;
    cfg->lost_target_hold_ms = 200.0;
    cfg->no_target_mode = NO_TARGET_HOLD;
    cfg->scan_speed_deg_s = 80.0;
    cfg->scan_range_deg = 80.0;

    cfg->gimbal_max_rate_deg_s = 500.0;
    cfg->history_sec = 8.0;
    cfg->headless = 0;
}

static void print_usage(const char *bin) {
    printf("Usage: %s [options]\n", bin);
    printf("  --mode step|sine|zigzag\n");
    printf("  --sim-hz N\n");
    printf("  --tracker-hz N\n");
    printf("  --publish-hz N\n");
    printf("  --duration-sec N\n");
    printf("  --amp-deg N\n");
    printf("  --period-sec N\n");
    printf("  --step-interval-sec N\n");
    printf("  --obs-alpha N (0..1)\n");
    printf("  --meas-noise-deg N\n");
    printf("  --dropout-prob N (0..1)\n");
    printf("  --publish-only-on-new-frame 0|1\n");
    printf("  --require-fresh-observation 0|1\n");
    printf("  --coast-timeout-sec N\n");
    printf("  --bt-reuse-latched 0|1\n");
    printf("  --lost-target-hold-ms N\n");
    printf("  --no-target-mode hold|scan\n");
    printf("  --scan-speed-deg-s N\n");
    printf("  --scan-range-deg N\n");
    printf("  --gimbal-max-rate-deg-s N\n");
    printf("  --history-sec N\n");
    printf("  --headless 0|1\n");
    printf("Controls (GUI): SPACE pause, Q/A alpha +/-, W/S gimbal-rate +/-, ESC quit\n");
}

static int parse_args(SimConfig *cfg, const int argc, char **argv) {
    for (int i = 1; i < argc; ++i) {
        const char *k = argv[i];
        if (strcmp(k, "--help") == 0 || strcmp(k, "-h") == 0) {
            return 1;
        }
        if (i + 1 >= argc) {
            fprintf(stderr, "Missing value for %s\n", k);
            return -1;
        }
        const char *v = argv[++i];
        if (strcmp(k, "--mode") == 0) cfg->mode = parse_mode(v);
        else if (strcmp(k, "--sim-hz") == 0) cfg->sim_hz = atof(v);
        else if (strcmp(k, "--tracker-hz") == 0) cfg->tracker_hz = atof(v);
        else if (strcmp(k, "--publish-hz") == 0) cfg->publish_hz = atof(v);
        else if (strcmp(k, "--duration-sec") == 0) cfg->duration_sec = atof(v);
        else if (strcmp(k, "--amp-deg") == 0) cfg->amp_deg = atof(v);
        else if (strcmp(k, "--period-sec") == 0) cfg->period_sec = atof(v);
        else if (strcmp(k, "--step-interval-sec") == 0) cfg->step_interval_sec = atof(v);
        else if (strcmp(k, "--obs-alpha") == 0) cfg->obs_alpha = atof(v);
        else if (strcmp(k, "--meas-noise-deg") == 0) cfg->meas_noise_deg = atof(v);
        else if (strcmp(k, "--dropout-prob") == 0) cfg->dropout_prob = atof(v);
        else if (strcmp(k, "--publish-only-on-new-frame") == 0) cfg->publish_only_on_new_frame = atoi(v);
        else if (strcmp(k, "--require-fresh-observation") == 0) cfg->require_fresh_observation = atoi(v);
        else if (strcmp(k, "--coast-timeout-sec") == 0) cfg->coast_timeout_sec = atof(v);
        else if (strcmp(k, "--bt-reuse-latched") == 0) cfg->bt_reuse_latched = atoi(v);
        else if (strcmp(k, "--lost-target-hold-ms") == 0) cfg->lost_target_hold_ms = atof(v);
        else if (strcmp(k, "--no-target-mode") == 0) cfg->no_target_mode = parse_no_target_mode(v);
        else if (strcmp(k, "--scan-speed-deg-s") == 0) cfg->scan_speed_deg_s = atof(v);
        else if (strcmp(k, "--scan-range-deg") == 0) cfg->scan_range_deg = atof(v);
        else if (strcmp(k, "--gimbal-max-rate-deg-s") == 0) cfg->gimbal_max_rate_deg_s = atof(v);
        else if (strcmp(k, "--history-sec") == 0) cfg->history_sec = atof(v);
        else if (strcmp(k, "--headless") == 0) cfg->headless = atoi(v);
        else {
            fprintf(stderr, "Unknown option: %s\n", k);
            return -1;
        }
    }
    cfg->sim_hz = clampd(cfg->sim_hz, 20.0, 2000.0);
    cfg->tracker_hz = clampd(cfg->tracker_hz, 1.0, cfg->sim_hz);
    cfg->publish_hz = clampd(cfg->publish_hz, 1.0, 5000.0);
    cfg->duration_sec = clampd(cfg->duration_sec, 1.0, 1200.0);
    cfg->obs_alpha = clampd(cfg->obs_alpha, 0.001, 1.0);
    cfg->dropout_prob = clampd(cfg->dropout_prob, 0.0, 1.0);
    cfg->coast_timeout_sec = clampd(cfg->coast_timeout_sec, 0.0, 5.0);
    cfg->gimbal_max_rate_deg_s = clampd(cfg->gimbal_max_rate_deg_s, 1.0, 5000.0);
    cfg->history_sec = clampd(cfg->history_sec, 1.0, 30.0);
    return 0;
}

static double target_yaw_deg(const SimConfig *cfg, const double t) {
    if (cfg->mode == MODE_SINE) {
        const double w = 2.0 * PI_D / fmax(cfg->period_sec, 1e-6);
        return cfg->amp_deg * sin(w * t);
    }
    if (cfg->mode == MODE_ZIGZAG) {
        const double phase = fmod(t, fmax(cfg->period_sec, 1e-6)) / fmax(cfg->period_sec, 1e-6);
        const double tri = -(2.0 * fabs(2.0 * phase - 1.0) - 1.0);  // [-1, 1]
        return cfg->amp_deg * tri;
    }
    // MODE_STEP
    const int k = (int)floor(t / fmax(cfg->step_interval_sec, 1e-6));
    const double sign = (k % 2 == 0) ? 1.0 : -1.0;
    return cfg->amp_deg * sign;
}

static void add_step_event(SimState *st, const double t, const double target_yaw) {
    if (st->event_count >= MAX_EVENTS) return;
    StepEvent *e = &st->events[st->event_count++];
    e->start_t = t;
    e->target_yaw = target_yaw;
    e->in_band_since = -1.0;
    e->settled = false;
    e->settle_time_sec = -1.0;
}

static void push_sample(SimState *st, const Sample *s) {
    st->samples[st->write_idx] = *s;
    st->write_idx = (st->write_idx + 1) % MAX_SAMPLES;
    if (st->count < MAX_SAMPLES) st->count++;
}

static int estimate_lag_samples(const SimState *st, const SimConfig *cfg, const int max_shift) {
    const int n = st->count;
    if (n < 64) return 0;
    int best_shift = 0;
    double best_mse = 1e300;
    bool found = false;
    const int shift_hi = (max_shift < (n - 16)) ? max_shift : (n - 16);
    for (int shift = 0; shift <= shift_hi; ++shift) {
        double mse = 0.0;
        int m = 0;
        for (int j = shift; j < n; ++j) {
            const int idx_g = (st->write_idx - n + j + MAX_SAMPLES) % MAX_SAMPLES;
            const int idx_t = (st->write_idx - n + (j - shift) + MAX_SAMPLES) % MAX_SAMPLES;
            const double e = angle_diff_deg(st->samples[idx_t].target_yaw, st->samples[idx_g].gimbal_yaw);
            mse += e * e;
            m++;
        }
        if (m < 32) continue;
        mse /= (double)m;
        found = true;
        if (mse < best_mse) {
            best_mse = mse;
            best_shift = shift;
        }
    }
    (void)cfg;
    if (!found) return 0;
    return best_shift;
}

static int cmp_double(const void *a, const void *b) {
    const double da = *(const double *)a;
    const double db = *(const double *)b;
    if (da < db) return -1;
    if (da > db) return 1;
    return 0;
}

static double percentile95(const SimState *st) {
    if (st->err_count <= 0 || st->errors == NULL) return 0.0;
    double *tmp = (double *)malloc((size_t)st->err_count * sizeof(double));
    if (!tmp) return 0.0;
    for (int i = 0; i < st->err_count; ++i) tmp[i] = st->errors[i];
    qsort(tmp, (size_t)st->err_count, sizeof(double), cmp_double);
    const int idx = (int)floor(0.95 * (double)(st->err_count - 1));
    const double p = tmp[idx];
    free(tmp);
    return p;
}

static void print_summary(const SimState *st, const SimConfig *cfg) {
    const double mae = st->err_count > 0 ? st->mae_sum / (double)st->err_count : 0.0;
    const double p95 = percentile95(st);
    const int max_shift = (int)(cfg->sim_hz * 1.0);
    const int lag_samples = estimate_lag_samples(st, cfg, max_shift);
    const double lag_ms = 1000.0 * (double)lag_samples / cfg->sim_hz;

    int settled_cnt = 0;
    double settled_sum = 0.0;
    for (int i = 0; i < st->event_count; ++i) {
        if (st->events[i].settled) {
            settled_cnt++;
            settled_sum += st->events[i].settle_time_sec;
        }
    }
    const double mean_settle = settled_cnt > 0 ? settled_sum / (double)settled_cnt : -1.0;

    printf("\n===== aa_response_sim summary =====\n");
    printf("samples=%d duration=%.3fs sim_hz=%.1f\n", st->count, st->sim_t, cfg->sim_hz);
    printf("mae_deg=%.3f p95_deg=%.3f max_abs_err_deg=%.3f\n", mae, p95, st->max_abs_err);
    printf("estimated_lag_ms=%.1f (shift=%d samples)\n", lag_ms, lag_samples);
    printf("step_events=%d settled=%d mean_settle_sec=%.3f\n",
           st->event_count, settled_cnt, mean_settle);
    printf("obs_alpha=%.3f dropout=%.3f gimbal_max_rate=%.1f deg/s\n",
           cfg->obs_alpha, cfg->dropout_prob, cfg->gimbal_max_rate_deg_s);
    printf("===================================\n");
}

static void update_settle_metrics(SimState *st, const double err_abs, const double t) {
    const double settle_band_deg = 3.0;
    const double settle_hold_sec = 0.10;
    (void)err_abs;
    for (int i = 0; i < st->event_count; ++i) {
        StepEvent *e = &st->events[i];
        if (e->settled) continue;
        const double event_err = fabs(angle_diff_deg(e->target_yaw, st->gimbal_yaw));
        if (event_err <= settle_band_deg) {
            if (e->in_band_since < 0.0) e->in_band_since = t;
            if (t - e->in_band_since >= settle_hold_sec) {
                e->settled = true;
                e->settle_time_sec = t - e->start_t;
            }
        } else {
            e->in_band_since = -1.0;
        }
    }
}

static void sim_step(SimState *st, const SimConfig *cfg, const double dt) {
    st->sim_t += dt;
    const double t = st->sim_t;
    const double target = target_yaw_deg(cfg, t);
    static double prev_target = 0.0;
    static bool prev_valid = false;
    if (cfg->mode == MODE_STEP) {
        if (prev_valid && fabs(target - prev_target) > 1e-6) {
            add_step_event(st, t, target);
        }
    }
    prev_target = target;
    prev_valid = true;

    bool has_measure = false;
    st->tracker_accum += dt;
    const double tracker_period = 1.0 / fmax(cfg->tracker_hz, 1e-6);
    while (st->tracker_accum >= tracker_period) {
        st->tracker_accum -= tracker_period;
        st->has_new_tracker_frame = true;
        has_measure = (frand01() >= cfg->dropout_prob);
        if (has_measure) {
            const double meas = wrap_deg(target + uniform_noise(cfg->meas_noise_deg));
            st->last_obs_t = t;
            if (!st->predictor_initialized) {
                st->predictor_yaw = meas;
                st->predictor_initialized = true;
            } else {
                const double d = angle_diff_deg(meas, st->predictor_yaw);
                st->predictor_yaw = wrap_deg(st->predictor_yaw + cfg->obs_alpha * d);
            }
        }
    }

    bool has_target_this_step = false;
    st->publish_accum += dt;
    const double pub_period = 1.0 / fmax(cfg->publish_hz, 1e-6);
    while (st->publish_accum >= pub_period) {
        st->publish_accum -= pub_period;
        bool should_pub = true;
        if (cfg->publish_only_on_new_frame && !st->has_new_tracker_frame) {
            should_pub = false;
        }
        st->has_new_tracker_frame = false;
        const bool fresh_obs = (t - st->last_obs_t) <= cfg->coast_timeout_sec;
        if (cfg->require_fresh_observation && !fresh_obs) {
            should_pub = false;
        }
        if (should_pub && st->predictor_initialized) {
            has_target_this_step = true;
            st->bt_cmd_yaw = st->predictor_yaw;
            st->latched_yaw = st->bt_cmd_yaw;
            st->has_latched = true;
            st->last_target_seen_t = t;
        }
    }

    if (!has_target_this_step) {
        const bool can_reuse = cfg->bt_reuse_latched &&
                               st->has_latched &&
                               (t - st->last_target_seen_t) * 1000.0 <= cfg->lost_target_hold_ms;
        if (can_reuse) {
            st->bt_cmd_yaw = st->latched_yaw;
        } else if (cfg->no_target_mode == NO_TARGET_HOLD) {
            st->bt_cmd_yaw = st->gimbal_yaw;
        } else {
            st->bt_cmd_yaw += st->scan_dir * cfg->scan_speed_deg_s * dt;
            if (st->bt_cmd_yaw > cfg->scan_range_deg) {
                st->bt_cmd_yaw = cfg->scan_range_deg;
                st->scan_dir = -1;
            } else if (st->bt_cmd_yaw < -cfg->scan_range_deg) {
                st->bt_cmd_yaw = -cfg->scan_range_deg;
                st->scan_dir = 1;
            }
        }
    }

    const double max_delta = cfg->gimbal_max_rate_deg_s * dt;
    double dg = angle_diff_deg(st->bt_cmd_yaw, st->gimbal_yaw);
    dg = clampd(dg, -max_delta, max_delta);
    st->gimbal_yaw = wrap_deg(st->gimbal_yaw + dg);

    const double err_abs = fabs(angle_diff_deg(target, st->gimbal_yaw));
    st->mae_sum += err_abs;
    st->max_abs_err = fmax(st->max_abs_err, err_abs);
    if (st->err_count < MAX_SAMPLES) st->errors[st->err_count++] = err_abs;
    update_settle_metrics(st, err_abs, t);

    Sample s;
    s.t = t;
    s.target_yaw = target;
    s.predictor_yaw = st->predictor_yaw;
    s.gimbal_yaw = st->gimbal_yaw;
    push_sample(st, &s);
}

static int y_from_yaw(const SDL_Rect *plot, const double yaw_deg, const double yaw_range_deg) {
    const double norm = clampd((yaw_deg + yaw_range_deg) / (2.0 * yaw_range_deg), 0.0, 1.0);
    return plot->y + (int)((1.0 - norm) * (double)plot->h);
}

static void draw_grid(SDL_Renderer *r, const SDL_Rect *plot) {
    SDL_SetRenderDrawColor(r, 40, 40, 40, 255);
    for (int i = 0; i <= 10; ++i) {
        const int x = plot->x + (plot->w * i) / 10;
        SDL_RenderDrawLine(r, x, plot->y, x, plot->y + plot->h);
    }
    for (int i = 0; i <= 8; ++i) {
        const int y = plot->y + (plot->h * i) / 8;
        SDL_RenderDrawLine(r, plot->x, y, plot->x + plot->w, y);
    }
    SDL_SetRenderDrawColor(r, 180, 180, 180, 255);
    SDL_RenderDrawRect(r, plot);
}

static void draw_signal(SDL_Renderer *r, const SimState *st, const SimConfig *cfg,
                        const SDL_Rect *plot, const int which,
                        const SDL_Color c) {
    if (st->count < 2) return;
    const double t_end = st->sim_t;
    const double t_start = t_end - cfg->history_sec;
    const double yaw_range = fmax(cfg->amp_deg * 1.4, 60.0);

    SDL_SetRenderDrawColor(r, c.r, c.g, c.b, c.a);

    bool have_prev = false;
    int px = 0, py = 0;
    for (int i = 0; i < st->count; ++i) {
        const int idx = (st->write_idx - st->count + i + MAX_SAMPLES) % MAX_SAMPLES;
        const Sample *s = &st->samples[idx];
        if (s->t < t_start) continue;
        const double xnorm = (s->t - t_start) / fmax(cfg->history_sec, 1e-6);
        const int x = plot->x + (int)(xnorm * (double)plot->w);
        double v = s->target_yaw;
        if (which == 1) v = s->predictor_yaw;
        if (which == 2) v = s->gimbal_yaw;
        const int y = y_from_yaw(plot, v, yaw_range);
        if (have_prev) SDL_RenderDrawLine(r, px, py, x, y);
        px = x;
        py = y;
        have_prev = true;
    }
}

static void render(SDL_Renderer *renderer, const SimState *st, const SimConfig *cfg,
                   const int win_w, const int win_h) {
    SDL_SetRenderDrawColor(renderer, 18, 18, 18, 255);
    SDL_RenderClear(renderer);

    SDL_Rect plot = {60, 50, win_w - 120, win_h - 120};
    draw_grid(renderer, &plot);
    draw_signal(renderer, st, cfg, &plot, 0, (SDL_Color){255, 80, 80, 255});    // target
    draw_signal(renderer, st, cfg, &plot, 1, (SDL_Color){255, 220, 80, 255});   // predictor
    draw_signal(renderer, st, cfg, &plot, 2, (SDL_Color){80, 220, 255, 255});   // gimbal

    SDL_RenderPresent(renderer);
}

int main(int argc, char **argv) {
    SimConfig cfg;
    set_default_config(&cfg);
    const int parse_ret = parse_args(&cfg, argc, argv);
    if (parse_ret == 1) {
        print_usage(argv[0]);
        return 0;
    }
    if (parse_ret != 0) {
        print_usage(argv[0]);
        return 2;
    }

    srand((unsigned int)SDL_GetTicks());

    SimState st;
    memset(&st, 0, sizeof(st));
    st.scan_dir = 1;
    st.last_obs_t = -1e9;
    st.last_target_seen_t = -1e9;
    st.paused = false;
    st.quit = false;
    st.errors = (double *)malloc((size_t)MAX_SAMPLES * sizeof(double));
    if (!st.errors) {
        fprintf(stderr, "Failed to allocate error buffer.\n");
        return 1;
    }

    printf("aa_response_sim started.\n");
    printf("mode=%s sim_hz=%.1f tracker_hz=%.1f publish_hz=%.1f obs_alpha=%.3f\n",
           (cfg.mode == MODE_STEP ? "step" : (cfg.mode == MODE_SINE ? "sine" : "zigzag")),
           cfg.sim_hz, cfg.tracker_hz, cfg.publish_hz, cfg.obs_alpha);

    SDL_Window *window = NULL;
    SDL_Renderer *renderer = NULL;
    int win_w = 1280;
    int win_h = 720;

    if (!cfg.headless) {
        if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
            fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
            free(st.errors);
            return 1;
        }
        window = SDL_CreateWindow("aa_response_sim",
                                  SDL_WINDOWPOS_CENTERED,
                                  SDL_WINDOWPOS_CENTERED,
                                  win_w, win_h, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
        if (!window) {
            fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
            SDL_Quit();
            free(st.errors);
            return 1;
        }
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
        if (!renderer) {
            fprintf(stderr, "SDL_CreateRenderer failed: %s\n", SDL_GetError());
            SDL_DestroyWindow(window);
            SDL_Quit();
            free(st.errors);
            return 1;
        }
    } else {
        // Timer subsystem only for headless pacing.
        if (SDL_Init(SDL_INIT_TIMER) != 0) {
            fprintf(stderr, "SDL_Init(timer) failed: %s\n", SDL_GetError());
            free(st.errors);
            return 1;
        }
    }

    const double sim_dt = 1.0 / cfg.sim_hz;
    Uint64 perf_prev = SDL_GetPerformanceCounter();
    const double perf_freq = (double)SDL_GetPerformanceFrequency();
    double accum = 0.0;
    double last_log_t = 0.0;

    while (!st.quit) {
        if (!cfg.headless) {
            SDL_Event e;
            while (SDL_PollEvent(&e)) {
                if (e.type == SDL_QUIT) st.quit = true;
                if (e.type == SDL_WINDOWEVENT && e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
                    win_w = e.window.data1;
                    win_h = e.window.data2;
                }
                if (e.type == SDL_KEYDOWN) {
                    const SDL_Keycode kc = e.key.keysym.sym;
                    if (kc == SDLK_ESCAPE) st.quit = true;
                    else if (kc == SDLK_SPACE) st.paused = !st.paused;
                    else if (kc == SDLK_q) cfg.obs_alpha = clampd(cfg.obs_alpha + 0.02, 0.001, 1.0);
                    else if (kc == SDLK_a) cfg.obs_alpha = clampd(cfg.obs_alpha - 0.02, 0.001, 1.0);
                    else if (kc == SDLK_w) cfg.gimbal_max_rate_deg_s = clampd(cfg.gimbal_max_rate_deg_s + 40.0, 1.0, 5000.0);
                    else if (kc == SDLK_s) cfg.gimbal_max_rate_deg_s = clampd(cfg.gimbal_max_rate_deg_s - 40.0, 1.0, 5000.0);
                }
            }
        }

        const Uint64 perf_now = SDL_GetPerformanceCounter();
        const double frame_dt = (double)(perf_now - perf_prev) / perf_freq;
        perf_prev = perf_now;

        if (!st.paused) {
            accum += frame_dt;
            while (accum >= sim_dt) {
                sim_step(&st, &cfg, sim_dt);
                accum -= sim_dt;
                if (st.sim_t >= cfg.duration_sec) {
                    st.quit = true;
                    break;
                }
            }
        }

        if (st.sim_t - last_log_t >= 1.0) {
            last_log_t = st.sim_t;
            const double mae = st.err_count > 0 ? st.mae_sum / (double)st.err_count : 0.0;
            const int lag_shift = estimate_lag_samples(&st, &cfg, (int)(cfg.sim_hz * 0.6));
            const double lag_ms = 1000.0 * (double)lag_shift / cfg.sim_hz;
            printf("t=%.1fs alpha=%.3f gimbal_rate=%.1f mae=%.2f lag~%.1fms\n",
                   st.sim_t, cfg.obs_alpha, cfg.gimbal_max_rate_deg_s, mae, lag_ms);
        }

        if (!cfg.headless) {
            render(renderer, &st, &cfg, win_w, win_h);
        } else {
            SDL_Delay(1);
        }
    }

    print_summary(&st, &cfg);

    if (!cfg.headless) {
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
    }
    SDL_Quit();
    free(st.errors);
    return 0;
}
