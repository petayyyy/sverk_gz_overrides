// Compile the actual pinned POSIX HRT driver into this translation unit.
// Timer wakeups and semaphores are deterministic, single-threaded test seams.
// No copy of the callout scheduling algorithm is maintained here.
#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <set>
#include PX4_HRT_SOURCE
#include PX4_QUEUE_SOURCE
#include PX4_ADD_AFTER_SOURCE
#include PX4_ADD_LAST_SOURCE

int px4_sem_init(px4_sem_t *s, int, unsigned value) { s->value = value; return 0; }
int px4_sem_wait(px4_sem_t *s) { if (--s->value < 0) std::abort(); return 0; }
int px4_sem_post(px4_sem_t *s) { ++s->value; return 0; }
void hrt_work_cancel(work_s *) {}
int hrt_work_queue(work_s *, worker_t, void *, uint32_t) { return 0; }
void px4_log_modulename(int, const char *, const char *, ...) {}

struct Fixture { hrt_call calibration{}; hrt_call battery{}; unsigned calibrations=0; unsigned batteries=0; };
void calibration_tick(void *arg)
{
    auto &f = *static_cast<Fixture *>(arg);
    if (++f.calibrations == 1) {
        // GyroCalibration::Run changes its interval at arming. A work-queue
        // thread can finish this before the original timer callback returns.
        hrt_call_every(&f.calibration, 0, 10000000, calibration_tick, &f);
    }
}
void battery_tick(void *arg) { ++static_cast<Fixture *>(arg)->batteries; }
int main()
{
    hrt_init();
    Fixture f;
    hrt_call_every(&f.battery, 10000, 10000, battery_tick, &f);
    hrt_call_every(&f.calibration, 0, 100000, calibration_tick, &f);
    hrt_call_invoke();
    std::set<const sq_entry_t *> entries;
    for (auto *e=callout_queue.head; e; e=e->flink) {
        if (!entries.insert(e).second) {
            std::fputs("FAIL: periodic reschedule created a timer-queue cycle\n",stderr);
            return 1;
        }
    }
    if (entries.size()!=2 || !entries.count(&f.battery.link)) {
        std::fputs("FAIL: periodic reschedule lost the independent battery timer\n",stderr);
        return 1;
    }
    usleep(15000);
    hrt_call_invoke();
    if (!f.batteries) { std::fputs("FAIL: independent battery timer stopped\n",stderr); return 1; }
    hrt_cancel(&f.calibration);
    hrt_cancel(&f.battery);
    std::puts("PASS: interval change retains an acyclic queue and the independent battery timer");
}
