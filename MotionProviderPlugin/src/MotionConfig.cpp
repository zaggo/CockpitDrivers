#include "MotionConfig.h"
#include "toml.hpp"
#include <cstdlib>

std::string MotionConfig::defaultPath() {
    const char* home = std::getenv("HOME");
    return home ? std::string(home) + "/.motionprovider.toml"
                : std::string(".motionprovider.toml");
}

StewartGeometry MotionConfig::loadGeometry(const std::string& path) {
    StewartGeometry g = StewartGeometry::defaults();

    toml::table tbl;
    try {
        tbl = toml::parse_file(path);
    } catch (const toml::parse_error&) {
        return g;  // missing or invalid file -> full defaults
    }

    auto geo = tbl["geometry"].as_table();
    if (geo) {
        if (auto v = (*geo)["base_radius_mm"].value<double>())     g.baseRadius = *v;
        if (auto v = (*geo)["platform_radius_mm"].value<double>()) g.platformRadius = *v;
        if (auto v = (*geo)["horn_length_mm"].value<double>())     g.hornLength = *v;
        if (auto v = (*geo)["rod_length_mm"].value<double>())      g.rodLength = *v;

        auto readArr6d = [](const toml::table& t, const char* key, double out[6]) {
            if (auto arr = t[key].as_array()) {
                for (int i = 0; i < 6 && i < static_cast<int>(arr->size()); ++i)
                    if (auto v = arr->get(i)->value<double>()) out[i] = *v;
            }
        };
        auto readArr6i = [](const toml::table& t, const char* key, int out[6]) {
            if (auto arr = t[key].as_array()) {
                for (int i = 0; i < 6 && i < static_cast<int>(arr->size()); ++i)
                    if (auto v = arr->get(i)->value<int64_t>()) out[i] = static_cast<int>(*v);
            }
        };
        readArr6d(*geo, "base_angle_deg",   g.phiDeg);
        readArr6d(*geo, "anchor_angle_deg", g.psiDeg);
        readArr6d(*geo, "horn_azimuth_deg", g.betaDeg);
        readArr6i(*geo, "bff_actuator",     g.bff);
    }

    auto servo = tbl["servo"].as_table();
    if (servo) {
        if (auto v = (*servo)["angle_at_full_scale_deg"].value<double>()) g.angleAtFullScale = *v;
        if (auto v = (*servo)["demand_home"].value<int64_t>()) g.demandHome = static_cast<int>(*v);
        if (auto v = (*servo)["demand_max"].value<int64_t>())  g.demandMax  = static_cast<int>(*v);
    }

    return g;
}
