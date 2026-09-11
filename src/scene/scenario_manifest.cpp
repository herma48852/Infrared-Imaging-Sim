#include "ir_sim/scene/scenario_manifest.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <variant>

namespace ir_sim::scene {

std::string to_string(TargetType type) {
    switch (type) {
        case TargetType::MilitaryVehicle: return "military_vehicle";
        case TargetType::CivilianVehicle: return "civilian_vehicle";
        case TargetType::Bird:            return "bird";
        case TargetType::DroneQuadcopter: return "drone_quadcopter";
        case TargetType::Personnel:       return "personnel";
    }
    return "unknown";
}

TargetType target_type_from_string(std::string_view str) {
    if (str == "military_vehicle") return TargetType::MilitaryVehicle;
    if (str == "civilian_vehicle") return TargetType::CivilianVehicle;
    if (str == "bird")            return TargetType::Bird;
    if (str == "drone_quadcopter") return TargetType::DroneQuadcopter;
    if (str == "personnel")       return TargetType::Personnel;
    return TargetType::MilitaryVehicle;
}

TargetKinematicState TargetDefinition::evaluate_at_time(float time_sec) const {
    TargetKinematicState state{};

    // Calculate composite thermal properties
    float total_area = 0.0f;
    float sum_temp = 0.0f;
    float sum_emis = 0.0f;

    for (const auto& z : thermal_zones) {
        const float a = std::max(0.001f, z.relative_area);
        total_area += a;
        sum_temp += a * z.temp_k;
        sum_emis += a * z.emissivity;
    }

    if (total_area > 0.0f) {
        state.composite_temp_k = sum_temp / total_area;
        state.composite_emissivity = sum_emis / total_area;
    } else {
        state.composite_temp_k = 295.0f;
        state.composite_emissivity = 0.90f;
    }

    if (waypoints.empty()) {
        return state;
    }

    if (waypoints.size() == 1 || time_sec <= waypoints.front().time_sec) {
        state.position = waypoints.front().position;
        state.velocity = {0.0f, 0.0f, 0.0f};
        state.yaw_deg = waypoints.front().yaw_deg;
        return state;
    }

    if (time_sec >= waypoints.back().time_sec) {
        state.position = waypoints.back().position;
        state.velocity = {0.0f, 0.0f, 0.0f};
        state.yaw_deg = waypoints.back().yaw_deg;
        return state;
    }

    // Binary search / scan for segment
    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
        const auto& w0 = waypoints[i];
        const auto& w1 = waypoints[i + 1];

        if (time_sec >= w0.time_sec && time_sec <= w1.time_sec) {
            const float dt = w1.time_sec - w0.time_sec;
            const float alpha = (dt > 1.0e-5f) ? ((time_sec - w0.time_sec) / dt) : 0.0f;

            state.position = {
                w0.position.x + alpha * (w1.position.x - w0.position.x),
                w0.position.y + alpha * (w1.position.y - w0.position.y),
                w0.position.z + alpha * (w1.position.z - w0.position.z)
            };

            if (dt > 1.0e-5f) {
                state.velocity = {
                    (w1.position.x - w0.position.x) / dt,
                    (w1.position.y - w0.position.y) / dt,
                    (w1.position.z - w0.position.z) / dt
                };
            }

            // Interpolate yaw angle smoothly
            float dyaw = w1.yaw_deg - w0.yaw_deg;
            while (dyaw > 180.0f) dyaw -= 360.0f;
            while (dyaw < -180.0f) dyaw += 360.0f;
            state.yaw_deg = w0.yaw_deg + alpha * dyaw;
            break;
        }
    }

    return state;
}

// =========================================================================
// Lightweight C++20 JSON Parser & Serializer
// =========================================================================

namespace json {

enum class Type { Null, Bool, Number, String, Array, Object };

struct Value;
using Object = std::map<std::string, Value>;
using Array = std::vector<Value>;

struct Value {
    Type type{Type::Null};
    std::variant<std::monostate, bool, double, std::string, Array, Object> data;

    Value() = default;
    Value(bool b) : type(Type::Bool), data(b) {}
    Value(double d) : type(Type::Number), data(d) {}
    Value(int i) : type(Type::Number), data(static_cast<double>(i)) {}
    Value(uint32_t u) : type(Type::Number), data(static_cast<double>(u)) {}
    Value(float f) : type(Type::Number), data(static_cast<double>(f)) {}
    Value(std::string s) : type(Type::String), data(std::move(s)) {}
    Value(const char* s) : type(Type::String), data(std::string(s)) {}
    Value(Array a) : type(Type::Array), data(std::move(a)) {}
    Value(Object o) : type(Type::Object), data(std::move(o)) {}

    [[nodiscard]] double as_number(double fallback = 0.0) const {
        if (std::holds_alternative<double>(data)) return std::get<double>(data);
        return fallback;
    }

    [[nodiscard]] float as_float(float fallback = 0.0f) const {
        return static_cast<float>(as_number(fallback));
    }

    [[nodiscard]] int as_int(int fallback = 0) const {
        return static_cast<int>(as_number(fallback));
    }

    [[nodiscard]] uint32_t as_uint32(uint32_t fallback = 0) const {
        return static_cast<uint32_t>(as_number(fallback));
    }

    [[nodiscard]] const std::string& as_string(const std::string& fallback = "") const {
        static const std::string empty;
        if (std::holds_alternative<std::string>(data)) return std::get<std::string>(data);
        return fallback.empty() ? empty : fallback;
    }

    [[nodiscard]] const Object& as_object() const {
        static const Object empty;
        if (std::holds_alternative<Object>(data)) return std::get<Object>(data);
        return empty;
    }

    [[nodiscard]] const Array& as_array() const {
        static const Array empty;
        if (std::holds_alternative<Array>(data)) return std::get<Array>(data);
        return empty;
    }

    [[nodiscard]] bool contains(const std::string& key) const {
        if (!std::holds_alternative<Object>(data)) return false;
        return std::get<Object>(data).find(key) != std::get<Object>(data).end();
    }

    [[nodiscard]] const Value& operator[](const std::string& key) const {
        static const Value null_val;
        if (!std::holds_alternative<Object>(data)) return null_val;
        const auto& obj = std::get<Object>(data);
        auto it = obj.find(key);
        if (it != obj.end()) return it->second;
        return null_val;
    }
};

class Parser {
public:
    explicit Parser(std::string_view text) : src_(text) {}

    std::optional<Value> parse() {
        skip_whitespace();
        if (pos_ >= src_.size()) return std::nullopt;
        return parse_value();
    }

private:
    std::string_view src_;
    size_t pos_{0};

    void skip_whitespace() {
        while (pos_ < src_.size()) {
            char c = src_[pos_];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
                pos_++;
            } else if (c == '/' && pos_ + 1 < src_.size() && src_[pos_ + 1] == '/') {
                // Single line comment
                pos_ += 2;
                while (pos_ < src_.size() && src_[pos_] != '\n') pos_++;
            } else {
                break;
            }
        }
    }

    std::optional<Value> parse_value() {
        skip_whitespace();
        if (pos_ >= src_.size()) return std::nullopt;

        char c = src_[pos_];
        if (c == '{') return parse_object();
        if (c == '[') return parse_array();
        if (c == '"') return parse_string();
        if (c == 't' || c == 'f') return parse_bool();
        if (c == 'n') return parse_null();
        if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) return parse_number();

        return std::nullopt;
    }

    std::optional<Value> parse_object() {
        pos_++; // skip '{'
        Object obj;

        while (true) {
            skip_whitespace();
            if (pos_ >= src_.size()) return std::nullopt;
            if (src_[pos_] == '}') {
                pos_++;
                return Value(std::move(obj));
            }

            auto key_val = parse_string();
            if (!key_val) return std::nullopt;
            std::string key = key_val->as_string();

            skip_whitespace();
            if (pos_ >= src_.size() || src_[pos_] != ':') return std::nullopt;
            pos_++; // skip ':'

            auto val = parse_value();
            if (!val) return std::nullopt;
            obj[key] = std::move(*val);

            skip_whitespace();
            if (pos_ >= src_.size()) return std::nullopt;
            if (src_[pos_] == ',') {
                pos_++;
            } else if (src_[pos_] == '}') {
                pos_++;
                return Value(std::move(obj));
            } else {
                return std::nullopt;
            }
        }
    }

    std::optional<Value> parse_array() {
        pos_++; // skip '['
        Array arr;

        while (true) {
            skip_whitespace();
            if (pos_ >= src_.size()) return std::nullopt;
            if (src_[pos_] == ']') {
                pos_++;
                return Value(std::move(arr));
            }

            auto val = parse_value();
            if (!val) return std::nullopt;
            arr.push_back(std::move(*val));

            skip_whitespace();
            if (pos_ >= src_.size()) return std::nullopt;
            if (src_[pos_] == ',') {
                pos_++;
            } else if (src_[pos_] == ']') {
                pos_++;
                return Value(std::move(arr));
            } else {
                return std::nullopt;
            }
        }
    }

    std::optional<Value> parse_string() {
        if (pos_ >= src_.size() || src_[pos_] != '"') return std::nullopt;
        pos_++; // skip opening quote

        std::string res;
        while (pos_ < src_.size()) {
            char c = src_[pos_++];
            if (c == '"') {
                return Value(res);
            }
            if (c == '\\' && pos_ < src_.size()) {
                char esc = src_[pos_++];
                switch (esc) {
                    case '"': res += '"'; break;
                    case '\\': res += '\\'; break;
                    case '/': res += '/'; break;
                    case 'b': res += '\b'; break;
                    case 'f': res += '\f'; break;
                    case 'n': res += '\n'; break;
                    case 'r': res += '\r'; break;
                    case 't': res += '\t'; break;
                    default: res += esc; break;
                }
            } else {
                res += c;
            }
        }
        return std::nullopt;
    }

    std::optional<Value> parse_number() {
        size_t start = pos_;
        if (src_[pos_] == '-') pos_++;
        while (pos_ < src_.size() && (std::isdigit(static_cast<unsigned char>(src_[pos_])) ||
               src_[pos_] == '.' || src_[pos_] == 'e' || src_[pos_] == 'E' ||
               src_[pos_] == '+' || src_[pos_] == '-')) {
            pos_++;
        }
        std::string num_str(src_.substr(start, pos_ - start));
        try {
            double d = std::stod(num_str);
            return Value(d);
        } catch (...) {
            return std::nullopt;
        }
    }

    std::optional<Value> parse_bool() {
        if (src_.substr(pos_, 4) == "true") {
            pos_ += 4;
            return Value(true);
        }
        if (src_.substr(pos_, 5) == "false") {
            pos_ += 5;
            return Value(false);
        }
        return std::nullopt;
    }

    std::optional<Value> parse_null() {
        if (src_.substr(pos_, 4) == "null") {
            pos_ += 4;
            return Value();
        }
        return std::nullopt;
    }
};

} // namespace json

// =========================================================================
// Manifest Serialization & Deserialization
// =========================================================================

std::string ScenarioManifest::to_json() const {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3);

    ss << "{\n";
    ss << "  \"scenario_name\": \"" << scenario_name << "\",\n";
    ss << "  \"description\": \"" << description << "\",\n";
    ss << "  \"duration_sec\": " << duration_sec << ",\n";

    // Environment
    ss << "  \"environment\": {\n";
    ss << "    \"ambient_temp_k\": " << environment.ambient_temp_k << ",\n";
    ss << "    \"solar_irradiance_w_m2\": " << environment.solar_irradiance_w_m2 << ",\n";
    ss << "    \"visibility_km\": " << environment.visibility_km << ",\n";
    ss << "    \"wind_vector_mps\": [" << environment.wind_vector_mps.x << ", "
       << environment.wind_vector_mps.y << ", " << environment.wind_vector_mps.z << "],\n";
    ss << "    \"time_of_day_hours\": " << environment.time_of_day_hours << "\n";
    ss << "  },\n";

    // Terrain
    ss << "  \"terrain\": {\n";
    ss << std::setprecision(6);
    ss << "    \"origin_lat\": " << terrain.origin_lat << ",\n";
    ss << "    \"origin_lon\": " << terrain.origin_lon << ",\n";
    ss << std::setprecision(2);
    ss << "    \"size_x_m\": " << terrain.size_x_m << ",\n";
    ss << "    \"size_y_m\": " << terrain.size_y_m << ",\n";
    ss << "    \"base_elevation_m\": " << terrain.base_elevation_m << ",\n";
    ss << "    \"terrain_type\": \"" << terrain.terrain_type << "\"\n";
    ss << "  },\n";

    // Targets
    ss << "  \"targets\": [\n";
    for (size_t i = 0; i < targets.size(); ++i) {
        const auto& t = targets[i];
        ss << "    {\n";
        ss << "      \"id\": " << t.id << ",\n";
        ss << "      \"name\": \"" << t.name << "\",\n";
        ss << "      \"type\": \"" << to_string(t.type) << "\",\n";
        ss << "      \"dimensions\": [" << t.dimensions.x << ", " << t.dimensions.y << ", " << t.dimensions.z << "],\n";

        // Thermal Zones
        ss << "      \"thermal_zones\": [\n";
        for (size_t z = 0; z < t.thermal_zones.size(); ++z) {
            const auto& tz = t.thermal_zones[z];
            ss << "        { \"name\": \"" << tz.name << "\", \"temp_k\": " << tz.temp_k
               << ", \"emissivity\": " << tz.emissivity << ", \"relative_area\": " << tz.relative_area << " }";
            if (z + 1 < t.thermal_zones.size()) ss << ",";
            ss << "\n";
        }
        ss << "      ],\n";

        // Waypoints
        ss << "      \"waypoints\": [\n";
        for (size_t w = 0; w < t.waypoints.size(); ++w) {
            const auto& wp = t.waypoints[w];
            ss << "        { \"time_sec\": " << wp.time_sec << ", \"pos\": ["
               << wp.position.x << ", " << wp.position.y << ", " << wp.position.z << "], \"speed_mps\": "
               << wp.speed_mps << ", \"yaw_deg\": " << wp.yaw_deg << " }";
            if (w + 1 < t.waypoints.size()) ss << ",";
            ss << "\n";
        }
        ss << "      ]\n";

        ss << "    }";
        if (i + 1 < targets.size()) ss << ",";
        ss << "\n";
    }
    ss << "  ]\n";
    ss << "}\n";

    return ss.str();
}

bool ScenarioManifest::save_to_file(const std::string& filepath) const {
    std::ofstream out(filepath);
    if (!out.is_open()) return false;
    out << to_json();
    return true;
}

std::optional<ScenarioManifest> ScenarioManifest::from_json(std::string_view json_str) {
    json::Parser parser(json_str);
    auto root_opt = parser.parse();
    if (!root_opt || root_opt->type != json::Type::Object) {
        return std::nullopt;
    }

    const auto& root = *root_opt;
    ScenarioManifest manifest;

    manifest.scenario_name = root["scenario_name"].as_string("unnamed_scenario");
    manifest.description = root["description"].as_string("");
    manifest.duration_sec = root["duration_sec"].as_float(60.0f);

    // Environment
    const auto& env = root["environment"];
    if (env.type == json::Type::Object) {
        manifest.environment.ambient_temp_k = env["ambient_temp_k"].as_float(288.15f);
        manifest.environment.solar_irradiance_w_m2 = env["solar_irradiance_w_m2"].as_float(200.0f);
        manifest.environment.visibility_km = env["visibility_km"].as_float(15.0f);
        manifest.environment.time_of_day_hours = env["time_of_day_hours"].as_float(14.0f);

        const auto& wind = env["wind_vector_mps"].as_array();
        if (wind.size() >= 3) {
            manifest.environment.wind_vector_mps = {
                wind[0].as_float(), wind[1].as_float(), wind[2].as_float()
            };
        }
    }

    // Terrain
    const auto& terr = root["terrain"];
    if (terr.type == json::Type::Object) {
        manifest.terrain.origin_lat = terr["origin_lat"].as_number(34.0522);
        manifest.terrain.origin_lon = terr["origin_lon"].as_number(-118.2437);
        manifest.terrain.size_x_m = terr["size_x_m"].as_float(2000.0f);
        manifest.terrain.size_y_m = terr["size_y_m"].as_float(2000.0f);
        manifest.terrain.base_elevation_m = terr["base_elevation_m"].as_float(100.0f);
        manifest.terrain.terrain_type = terr["terrain_type"].as_string("desert");
    }

    // Targets
    const auto& targets_arr = root["targets"].as_array();
    for (const auto& t_val : targets_arr) {
        TargetDefinition target;
        target.id = t_val["id"].as_uint32(1);
        target.name = t_val["name"].as_string("target");
        target.type = target_type_from_string(t_val["type"].as_string("military_vehicle"));

        const auto& dims = t_val["dimensions"].as_array();
        if (dims.size() >= 3) {
            target.dimensions = {dims[0].as_float(4.0f), dims[1].as_float(2.0f), dims[2].as_float(2.0f)};
        }

        // Thermal Zones
        const auto& zones_arr = t_val["thermal_zones"].as_array();
        for (const auto& z_val : zones_arr) {
            ThermalZone tz;
            tz.name = z_val["name"].as_string("zone");
            tz.temp_k = z_val["temp_k"].as_float(295.0f);
            tz.emissivity = z_val["emissivity"].as_float(0.90f);
            tz.relative_area = z_val["relative_area"].as_float(1.0f);
            target.thermal_zones.push_back(tz);
        }

        // Waypoints
        const auto& wp_arr = t_val["waypoints"].as_array();
        for (const auto& w_val : wp_arr) {
            Waypoint wp;
            wp.time_sec = w_val["time_sec"].as_float(0.0f);
            wp.speed_mps = w_val["speed_mps"].as_float(0.0f);
            wp.yaw_deg = w_val["yaw_deg"].as_float(0.0f);

            const auto& pos_arr = w_val["pos"].as_array();
            if (pos_arr.size() >= 3) {
                wp.position = {pos_arr[0].as_float(), pos_arr[1].as_float(), pos_arr[2].as_float()};
            }
            target.waypoints.push_back(wp);
        }

        manifest.targets.push_back(std::move(target));
    }

    return manifest;
}

std::optional<ScenarioManifest> ScenarioManifest::load_from_file(const std::string& filepath) {
    std::ifstream in(filepath);
    if (!in.is_open()) return std::nullopt;

    std::ostringstream ss;
    ss << in.rdbuf();
    return from_json(ss.str());
}

} // namespace ir_sim::scene
