#pragma once
#include "wled.h"
#include <math.h>

extern time_t localTime;
extern WS2812FX strip;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class Usermod_sun_moon_fx : public Usermod {
private:
  static const char _name[];
  static const char _enabled[];
  static const char _map_width_cm[];
  static const char _map_height_cm[];
  static const char _upper_first_led[];
  static const char _upper_last_led[];
  static const char _lower_first_led[];
  static const char _lower_last_led[];
  static const char _show_sun[];
  static const char _show_moon[];
  static const char _local_night_mode[];
  static const char _realistic_mode[];

  static const char _data_FX_MODE_SUNMOON[] PROGMEM;

public:
  static bool enabled;
  static byte map_width_cm;
  static byte map_height_cm;
  static int upper_first_led;
  static int upper_last_led;
  static int lower_first_led;
  static int lower_last_led;
  static bool show_sun;
  static bool show_moon;
  static bool local_night_mode;
  static bool realistic_mode;
  static unsigned long lastCheck;
  static int8_t tzOffset;

  // ------------------ Astronomical helpers ------------------
  static double unixToJD(time_t unix) { return (unix / 86400.0) + 2440587.5; }
  static double getGMST(double jd) {
    double d = jd - 2451545.0;
    double T = d / 36525.0;
    double gmst = 280.46061837 + 360.98564736629 * d +
                  0.000387933 * T * T - T * T * T / 38710000.0;
    return fmod(gmst, 360.0);
  }
  static double getSunDecl(double jd) {
    double n = fmod(jd - 2451545.0 + 10, 365.25);
    double angle = 2 * M_PI * (284 + n) / 365.25;
    return 23.45 * sin(angle);
  }
  static double getSunEclLon(double jd) {
    double d = jd - 2451545.0;
    double T = d / 36525.0;
    double L = 280.46646 + T * (36000.76983 + T * 0.0003032);
    L = fmod(L, 360.0);
    return L;
  }
  static double getSunLon(double jd) {
    double gmst = getGMST(jd);
    double L = getSunEclLon(jd);
    double lon = gmst - L;
    if (lon > 180) lon -= 360;
    else if (lon < -180) lon += 360;
    return lon;
  }
  static double getMoonEclLon(double jd) {
    double d = jd - 2451545.0;
    double L = 218.316 + 13.176396 * d;
    L += 6.289 * sin(13.176396 * d * M_PI / 180.0);
    return fmod(L, 360.0);
  }
  static double getMoonEclLat(double jd) {
    double d = jd - 2451545.0;
    double L = getMoonEclLon(jd);
    double B = 5.13 * sin(M_PI / 180 * (L - 218.32));
    return B;
  }
  static double getMoonLon(double jd) {
    double gmst = getGMST(jd);
    double moon_ecl_lon = getMoonEclLon(jd);
    double lon = gmst - moon_ecl_lon;
    if (lon > 180) lon -= 360;
    else if (lon < -180) lon += 360;
    return lon;
  }
  static double haversine(double lat1, double lon1, double lat2, double lon2) {
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dlat / 2) * sin(dlat / 2) +
               cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
               sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return c * 180.0 / M_PI;
  }

  static void ledToLatLon(int index, double& lat, double& lon) {
    bool in_upper = (index >= upper_first_led && index <= upper_last_led);
    bool in_lower = (index >= lower_first_led && index <= lower_last_led);
    if (!in_upper && !in_lower) { lat = 0; lon = 0; return; }

    int strip_offset = in_upper ? upper_first_led : lower_first_led;
    int strip_length = in_upper ?
      (upper_last_led - upper_first_led + 1) :
      (lower_last_led - lower_first_led + 1);

    if (strip_length <= 0) { lat = 0; lon = 0; return; }

    byte strip_width = map_width_cm;
    float y_rel = (index - strip_offset) / (float)(strip_length - 1);
    lat = in_upper ? (y_rel * 90.0) : (-90.0 + y_rel * 90.0);

    int col = (index - strip_offset) % strip_width;
    float x_rel = col / (float)(strip_width - 1);
    lon = -180.0 + (x_rel * 360.0);
  }

  static uint32_t addColors(uint32_t c1, uint32_t c2) {
    byte r1 = R(c1), g1 = G(c1), b1 = B(c1), w1 = W(c1);
    byte r2 = R(c2), g2 = G(c2), b2 = B(c2), w2 = W(c2);
    return RGBW32(
      min(255, (int)r1 + r2),
      min(255, (int)g1 + g2),
      min(255, (int)b1 + b2),
      min(255, (int)w1 + w2)
    );
  }

  // ------------------ Setup & Loop ------------------
  void setup() override {
    DEBUG_PRINTLN(F("Sun & Moon FX Usermod loaded"));
    if (enabled) {
      strip.addEffect(255, mode_sunMoon, (char*)_data_FX_MODE_SUNMOON);
    }
  }

  void loop() override {
    if (millis() - lastCheck > 30000UL) {
      lastCheck = millis();
      DEBUG_PRINTF("SunMoon FX active: %s\n", enabled ? "yes" : "no");
    }
  }

    void addToJsonInfo(JsonObject* root) {
    JsonObject sm = (*root).createNestedObject(F("sunmoonfx"));
    sm["enabled"] = enabled;
  }

  void addToConfig(JsonObject &root) {
    JsonObject top = root.createNestedObject(FPSTR(_name));
    top[FPSTR(_enabled)] = enabled;
    top[FPSTR(_map_width_cm)] = map_width_cm;
    top[FPSTR(_map_height_cm)] = map_height_cm;
    top[FPSTR(_upper_first_led)] = upper_first_led;
    top[FPSTR(_upper_last_led)] = upper_last_led;
    top[FPSTR(_lower_first_led)] = lower_first_led;
    top[FPSTR(_lower_last_led)] = lower_last_led;
    top[FPSTR(_show_sun)] = show_sun;
    top[FPSTR(_show_moon)] = show_moon;
    top[FPSTR(_local_night_mode)] = local_night_mode;
    top[FPSTR(_realistic_mode)] = realistic_mode;
  }

  bool readFromConfig(JsonObject &root) {
    JsonObject top = root[FPSTR(_name)];
    if (top.isNull()) return false;

    getJsonValue(top[FPSTR(_enabled)], enabled, true);
    getJsonValue(top[FPSTR(_map_width_cm)], map_width_cm, 80);
    getJsonValue(top[FPSTR(_map_height_cm)], map_height_cm, 40);
    getJsonValue(top[FPSTR(_upper_first_led)], upper_first_led, 200);
    getJsonValue(top[FPSTR(_upper_last_led)], upper_last_led, 399);
    getJsonValue(top[FPSTR(_lower_first_led)], lower_first_led, 0);
    getJsonValue(top[FPSTR(_lower_last_led)], lower_last_led, 199);
    getJsonValue(top[FPSTR(_show_sun)], show_sun, true);
    getJsonValue(top[FPSTR(_show_moon)], show_moon, true);
    getJsonValue(top[FPSTR(_local_night_mode)], local_night_mode, false);
    getJsonValue(top[FPSTR(_realistic_mode)], realistic_mode, false);

    return true;
  }

  uint16_t getId() override { return USERMOD_ID_USER_FX; }

  // --- Main FX Function ---
  static uint16_t mode_sunMoon() {
    static unsigned long lastUpdate = 0;
    if (!enabled || millis() - lastUpdate < 60000UL) return 50;
    lastUpdate = millis();

    time_t t = localTime;
    double jd = unixToJD(t);
    double sun_lat = getSunDecl(jd);
    double sun_lon = getSunLon(jd);
    double moon_lat = getMoonEclLat(jd);
    double moon_lon = getMoonLon(jd);

    // Local Night Logic
    bool showSun = show_sun;
    bool showMoon = show_moon;

    if (local_night_mode) {
      // It is 'night' when the Sun is over the horizon at localtime
      time_t utc = t - tzOffset * 3600;
      double jdLocal = unixToJD(utc);
      double lonLocal = 0; // middlepoint map
      double diff = fabs(sun_lon - lonLocal);
      bool isNight = (diff > 90 && diff < 270);
      if (isNight) {
        showSun = false;
        showMoon = true;
      } else {
        showSun = true;
        showMoon = false;
      }
    }

    for (int i = 0; i < SEGLEN; i++) {
      double lat, lon;
      ledToLatLon(SEGMENT.start + i, lat, lon);
      uint32_t color = 0;

      // --- Realistic mode daylight shading ---
      if (realistic_mode) {
        double lonDiff = fabs(lon - sun_lon);
        if (lonDiff > 180) lonDiff = 360 - lonDiff;
        double dayFactor = cos(lonDiff * M_PI / 180.0);
        uint8_t baseBri = (uint8_t)(constrain((dayFactor + 1.0) / 2.0 * 255, 0, 255));
        color = RGBW32(baseBri, baseBri * 0.8, baseBri * 0.5, 0);
      }

      // --- Sun rendering ---
      if (showSun) {
        double dist = haversine(lat, lon, sun_lat, sun_lon);
        if (dist < 45.0) {
          float fade = cos(dist * M_PI / 90.0);
          uint8_t bri = (uint8_t)(255 * fade);
          color = addColors(color, RGBW32(bri, bri * 0.7, 0, 0));
        }
      }

      // --- Moon rendering ---
      if (showMoon) {
        double dist = haversine(lat, lon, moon_lat, moon_lon);
        if (dist < 45.0) {
          float fade = cos(dist * M_PI / 90.0);
          uint8_t bri = (uint8_t)(255 * fade);
          color = addColors(color, RGBW32(bri, bri, bri, 0));
        }
      }

      SEGMENT.setPixelColor(i, color);
    }
    return 50;
  }
};


// --- Static variable definitions ---
const char Usermod_sun_moon_fx::_name[] PROGMEM = "sunmoonfx";
const char Usermod_sun_moon_fx::_enabled[] PROGMEM = "enabled";
const char Usermod_sun_moon_fx::_map_width_cm[] PROGMEM = "Map width in cm";
const char Usermod_sun_moon_fx::_map_height_cm[] PROGMEM = "Map height in cm";
const char Usermod_sun_moon_fx::_upper_first_led[] PROGMEM = "Upper first led";
const char Usermod_sun_moon_fx::_upper_last_led[] PROGMEM = "Upper last led";
const char Usermod_sun_moon_fx::_lower_first_led[] PROGMEM = "Lower first led";
const char Usermod_sun_moon_fx::_lower_last_led[] PROGMEM = "Lower last led";
const char Usermod_sun_moon_fx::_show_sun[] PROGMEM = "Show Sun";
const char Usermod_sun_moon_fx::_show_moon[] PROGMEM = "Show Moon";
const char Usermod_sun_moon_fx::_local_night_mode[] PROGMEM = "local night";
const char Usermod_sun_moon_fx::_realistic_mode[] PROGMEM = "realistic";

const char Usermod_sun_moon_fx::_data_FX_MODE_SUNMOON[] PROGMEM = "🌞 Sun & Moon FX@!;!;!;01";

bool Usermod_sun_moon_fx::enabled = true;
byte Usermod_sun_moon_fx::map_width_cm = 80;
byte Usermod_sun_moon_fx::map_height_cm = 40;
int Usermod_sun_moon_fx::upper_first_led = 200;
int Usermod_sun_moon_fx::upper_last_led = 399;
int Usermod_sun_moon_fx::lower_first_led = 0;
int Usermod_sun_moon_fx::lower_last_led = 199;
bool Usermod_sun_moon_fx::show_sun = true;
bool Usermod_sun_moon_fx::show_moon = true;
bool Usermod_sun_moon_fx::local_night_mode = false;
bool Usermod_sun_moon_fx::realistic_mode = false;
unsigned long Usermod_sun_moon_fx::lastCheck = 0;
int8_t Usermod_sun_moon_fx::tzOffset = 0;

// Register usermod with WLED
static Usermod_sun_moon_fx sunMoonUsermod;
REGISTER_USERMOD(sunMoonUsermod);
