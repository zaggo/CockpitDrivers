// Measures the platform's reachable surge/sway travel, bare and in the corner
// of the envelope the other channels already occupy. Links the real
// StewartKinematics -- there is deliberately no second implementation of the
// geometry that could drift from the plugin's.
#include "MotionConfig.h"
#include "StewartKinematics.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

bool reachable(const StewartKinematics& k, const Pose& p) {
    return k.solve(p).allReachable;
}

// Largest travel along `axis` in direction `dir` that still solves, searched in
// [0, hi] by bisection. Assumes `base` is reachable and that reachability is
// monotone along the axis -- true here: the legs run out of travel as the pose
// moves away from home, they do not come back.
double maxTravel(const StewartKinematics& k, const Pose& base,
                 float Pose::*axis, double dir, double hi) {
    Pose p = base;
    p.*axis = static_cast<float>(base.*axis + dir * hi);
    if (reachable(k, p)) return hi;      // never ran out inside the search range
    double lo = 0.0;
    for (int i = 0; i < 60; ++i) {
        const double mid = 0.5 * (lo + hi);
        p = base;
        p.*axis = static_cast<float>(base.*axis + dir * mid);
        if (reachable(k, p)) lo = mid; else hi = mid;
    }
    return lo;
}

// Largest symmetric roll=pitch (all else zero) that still solves, searched in
// [0, hi] by bisection. Same monotonicity assumption as maxTravel. This is a
// diagnostic on the theoretical clamp corner (roll/pitch at their combined
// per-axis maximum) -- report only, not an input to the chosen limits.
double maxSymmetricRollPitch(const StewartKinematics& k, double hi) {
    auto reachableAt = [&](double x) -> bool {
        Pose p;
        p.roll = static_cast<float>(x);
        p.pitch = static_cast<float>(x);
        return reachable(k, p);
    };
    if (reachableAt(hi)) return hi;      // never ran out inside the search range
    double lo = 0.0;
    for (int i = 0; i < 60; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (reachableAt(mid)) lo = mid; else hi = mid;
    }
    return lo;
}

void report(const StewartKinematics& k, const char* label, const Pose& base) {
    if (!reachable(k, base)) {
        std::printf("%-28s BASE POSE UNREACHABLE\n", label);
        return;
    }
    const double sPos = maxTravel(k, base, &Pose::surge, +1.0, 500.0);
    const double sNeg = maxTravel(k, base, &Pose::surge, -1.0, 500.0);
    const double yPos = maxTravel(k, base, &Pose::sway,  +1.0, 500.0);
    const double yNeg = maxTravel(k, base, &Pose::sway,  -1.0, 500.0);
    std::printf("%-28s surge +%7.2f / -%7.2f    sway +%7.2f / -%7.2f  (mm)\n",
                label, sPos, sNeg, yPos, yNeg);
}

// Parses "heave,roll,pitch,yaw" (mm, deg, deg, deg) into a base Pose with
// surge/sway left at zero -- the caller measures surge/sway travel around it.
bool parsePose(const std::string& csv, Pose* out) {
    std::vector<double> v;
    std::stringstream ss(csv);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        if (tok.empty()) return false;
        v.push_back(std::atof(tok.c_str()));
    }
    if (v.size() != 4) return false;
    out->heave = static_cast<float>(v[0]);
    out->roll  = static_cast<float>(v[1]);
    out->pitch = static_cast<float>(v[2]);
    out->yaw   = static_cast<float>(v[3]);
    return true;
}

// One sampled tick from a replay CSV, reduced to what report()/maxTravel need.
struct ReplayTick {
    long   row  = 0;      // 1-based data row (header excluded)
    double tsec = 0.0;
    Pose   pose;          // surge/sway left at 0 -- travel is measured around it
};

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::string field;
    std::stringstream ss(line);
    while (std::getline(ss, field, ',')) out.push_back(field);
    return out;
}

std::string trimCr(std::string s) {
    while (!s.empty() && (s.back() == '\r' || s.back() == '\n')) s.pop_back();
    return s;
}

// Reads a replay telemetry CSV by column name (t_sec, live_heave, live_roll,
// live_pitch, live_yaw) -- never by hard-coded index, so a Telemetry column
// reorder can't silently mis-read this. Samples every `stride`-th data row
// (1-based; stride 1 = every row).
bool loadReplayTicks(const std::string& path, int stride, std::vector<ReplayTick>* out,
                     std::string* err) {
    std::ifstream in(path);
    if (!in.is_open()) {
        *err = "could not open " + path;
        return false;
    }
    std::string headerLine;
    if (!std::getline(in, headerLine)) {
        *err = path + ": empty file";
        return false;
    }
    const std::vector<std::string> cols = splitCsv(trimCr(headerLine));
    int idxT = -1, idxH = -1, idxR = -1, idxP = -1, idxY = -1;
    for (size_t i = 0; i < cols.size(); ++i) {
        if      (cols[i] == "t_sec")      idxT = static_cast<int>(i);
        else if (cols[i] == "live_heave") idxH = static_cast<int>(i);
        else if (cols[i] == "live_roll")  idxR = static_cast<int>(i);
        else if (cols[i] == "live_pitch") idxP = static_cast<int>(i);
        else if (cols[i] == "live_yaw")   idxY = static_cast<int>(i);
    }
    if (idxT < 0 || idxH < 0 || idxR < 0 || idxP < 0 || idxY < 0) {
        *err = path + ": missing one of t_sec/live_heave/live_roll/live_pitch/live_yaw in header";
        return false;
    }
    const int need = std::max({idxT, idxH, idxR, idxP, idxY});

    std::string line;
    long row = 0;
    while (std::getline(in, line)) {
        ++row;
        if ((row - 1) % stride != 0) continue;
        line = trimCr(line);
        if (line.empty()) continue;
        const std::vector<std::string> f = splitCsv(line);
        if (static_cast<int>(f.size()) <= need) continue;
        ReplayTick t;
        t.row  = row;
        t.tsec = std::atof(f[idxT].c_str());
        t.pose.heave = static_cast<float>(std::atof(f[idxH].c_str()));
        t.pose.roll  = static_cast<float>(std::atof(f[idxR].c_str()));
        t.pose.pitch = static_cast<float>(std::atof(f[idxP].c_str()));
        t.pose.yaw   = static_cast<float>(std::atof(f[idxY].c_str()));
        out->push_back(t);
    }
    return true;
}

// One axis's measured travel at one sampled tick, kept with enough context to
// locate the worst case afterward.
struct TravelSample {
    double      val  = 0.0;
    std::string file;
    long        row  = 0;
    double      tsec = 0.0;
};

double percentile(std::vector<TravelSample>& sorted, double p) {
    if (sorted.empty()) return 0.0;
    size_t idx = static_cast<size_t>(std::lround(p * (sorted.size() - 1)));
    if (idx >= sorted.size()) idx = sorted.size() - 1;
    return sorted[idx].val;
}

// Prints min (with its file/row/t_sec so the worst case is locatable), the 1st
// percentile, and the median for one axis's samples. Sorts `samples` in place.
void printAxisStats(const char* axisLabel, std::vector<TravelSample>& samples) {
    if (samples.empty()) {
        std::printf("  %-7s no samples -- nothing loaded (empty CSV, all rows too short, "
                    "or every input failed to load)\n", axisLabel);
        return;
    }
    std::sort(samples.begin(), samples.end(),
              [](const TravelSample& a, const TravelSample& b) { return a.val < b.val; });
    const TravelSample& mn = samples.front();
    const double p01 = percentile(samples, 0.01);
    const double med = percentile(samples, 0.50);
    std::printf("  %-7s min %8.2f mm  (file=%-16s row=%-6ld t=%8.3f s)   p01 %8.2f mm   median %8.2f mm\n",
                axisLabel, mn.val, mn.file.c_str(), mn.row, mn.tsec, p01, med);
}

}  // namespace

int main(int argc, char** argv) {
    std::string cfgPath = "configuration.toml";
    std::string poseArg;
    bool havePose = false;
    std::vector<std::string> replayFiles;
    int stride = 1;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            cfgPath = argv[++i];
        } else if (std::strcmp(argv[i], "--pose") == 0 && i + 1 < argc) {
            poseArg = argv[++i];
            havePose = true;
        } else if (std::strcmp(argv[i], "--from-replay") == 0 && i + 1 < argc) {
            replayFiles.push_back(argv[++i]);
        } else if (std::strcmp(argv[i], "--stride") == 0 && i + 1 < argc) {
            stride = std::atoi(argv[++i]);
            if (stride < 1) stride = 1;
        }
    }

    bool loaded = false;
    const StewartGeometry geo = MotionConfig::loadGeometry(cfgPath, &loaded);
    std::printf("config: %s (%s)\n", cfgPath.c_str(), loaded ? "loaded" : "DEFAULTS -- file not read");
    StewartKinematics kin(geo);
    std::printf("home height: %.2f mm\n\n", kin.homeHeight());

    // Bare: an upper bound, never available in flight.
    report(kin, "home pose", Pose{});

    // In the corner: heave at its limit and tilt+rotational at their combined
    // per-axis limit, i.e. what the existing channels are already allowed to
    // occupy at the same time. This is the THEORETICAL clamp corner -- the two
    // channels' independent clamps stacked, not a flown operating point.
    for (double hs : {+1.0, -1.0}) {
        for (double as : {+1.0, -1.0}) {
            Pose c;
            c.heave = static_cast<float>(hs * 30.0);
            c.roll  = static_cast<float>(as * 14.0);
            c.pitch = static_cast<float>(as * 14.0);
            c.yaw   = static_cast<float>(as * 7.0);
            char label[64];
            std::snprintf(label, sizeof(label), "corner h%+.0f r/p%+.0f y%+.0f",
                          c.heave, c.roll, c.yaw);
            report(kin, label, c);
        }
    }

    // Diagnostic on that theoretical corner: how far the two stacked channels
    // could go together, symmetrically, before running out of legs. Report
    // only -- this does not feed the chosen limits.
    const double maxRP = maxSymmetricRollPitch(kin, 45.0);
    std::printf("\nlargest symmetric reachable roll=pitch (all else zero): %.2f deg\n", maxRP);

    // Empirical (or any other ad-hoc) base pose supplied on the command line,
    // so a candidate corner can be probed without recompiling it in.
    if (havePose) {
        Pose p;
        if (!parsePose(poseArg, &p)) {
            std::fprintf(stderr, "--pose expects \"heave,roll,pitch,yaw\" (mm,deg,deg,deg), got \"%s\"\n",
                         poseArg.c_str());
            return 1;
        }
        char label[64];
        std::snprintf(label, sizeof(label), "pose h%+.2f r%+.2f p%+.2f y%+.2f",
                      p.heave, p.roll, p.pitch, p.yaw);
        std::printf("\n");
        report(kin, label, p);
    }

    // Batch mode: surge/sway headroom over every (or every Nth) recorded pose
    // in one or more replay CSVs -- not a synthetic corner, the poses the
    // platform actually held. Reuses maxTravel; no second bisection.
    if (!replayFiles.empty()) {
        std::printf("\n--from-replay (stride %d)\n", stride);
        std::vector<TravelSample> allSPos, allSNeg, allYPos, allYNeg;
        for (const std::string& path : replayFiles) {
            std::vector<ReplayTick> ticks;
            std::string err;
            if (!loadReplayTicks(path, stride, &ticks, &err)) {
                std::fprintf(stderr, "--from-replay: %s\n", err.c_str());
                continue;
            }
            std::printf("\n%s (%zu sampled ticks)\n", path.c_str(), ticks.size());
            std::vector<TravelSample> sPos, sNeg, yPos, yNeg;
            sPos.reserve(ticks.size());
            sNeg.reserve(ticks.size());
            yPos.reserve(ticks.size());
            yNeg.reserve(ticks.size());
            for (const ReplayTick& t : ticks) {
                if (!reachable(kin, t.pose)) {
                    // The recorded "live" pose is clampToReachable's own output,
                    // so it is reachable by construction there; a handful of
                    // ticks re-solve as marginally unreachable here purely from
                    // the CSV round-trip's precision loss landing right on the
                    // boundary clampToReachable's bisection converged to. Warn
                    // (visible, locatable) but do not skip: dropping the tick
                    // would silently discard exactly the samples with the least
                    // headroom. maxTravel doesn't require a reachable base to
                    // answer sensibly -- reachability is monotone outward, so a
                    // base already at (or a hair past) the boundary just makes
                    // every trial pose unreachable too and the bisection
                    // converges to ~0 mm, which is the right answer: no room
                    // left at that instant.
                    std::fprintf(stderr, "  WARNING: row %ld t=%.3f is UNREACHABLE as a base pose "
                                         "(kept in stats; expect ~0 mm travel)\n", t.row, t.tsec);
                }
                TravelSample s;
                s.file = path;
                s.row  = t.row;
                s.tsec = t.tsec;
                s.val  = maxTravel(kin, t.pose, &Pose::surge, +1.0, 500.0); sPos.push_back(s);
                s.val  = maxTravel(kin, t.pose, &Pose::surge, -1.0, 500.0); sNeg.push_back(s);
                s.val  = maxTravel(kin, t.pose, &Pose::sway,  +1.0, 500.0); yPos.push_back(s);
                s.val  = maxTravel(kin, t.pose, &Pose::sway,  -1.0, 500.0); yNeg.push_back(s);
            }
            printAxisStats("surge+", sPos);
            printAxisStats("surge-", sNeg);
            printAxisStats("sway+",  yPos);
            printAxisStats("sway-",  yNeg);
            allSPos.insert(allSPos.end(), sPos.begin(), sPos.end());
            allSNeg.insert(allSNeg.end(), sNeg.begin(), sNeg.end());
            allYPos.insert(allYPos.end(), yPos.begin(), yPos.end());
            allYNeg.insert(allYNeg.end(), yNeg.begin(), yNeg.end());
        }
        std::printf("\ncombined (%zu files, %zu sampled ticks total)\n",
                     replayFiles.size(), allSPos.size());
        printAxisStats("surge+", allSPos);
        printAxisStats("surge-", allSNeg);
        printAxisStats("sway+",  allYPos);
        printAxisStats("sway-",  allYNeg);

        // Both directions pooled into one distribution per axis -- what a
        // symmetric surge_limit_mm/sway_limit_mm is actually sized against.
        std::vector<TravelSample> allSurge = allSPos;
        allSurge.insert(allSurge.end(), allSNeg.begin(), allSNeg.end());
        std::vector<TravelSample> allSway = allYPos;
        allSway.insert(allSway.end(), allYNeg.begin(), allYNeg.end());
        std::printf("\ncombined, both directions pooled (%zu samples each axis)\n", allSurge.size());
        printAxisStats("surge", allSurge);
        printAxisStats("sway",  allSway);
    }
    return 0;
}
