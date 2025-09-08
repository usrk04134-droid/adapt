#pragma once

#include <SQLiteCpp/Database.h>

#include <functional>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace weld_control {

struct WeldProgramLayer {
  int layer_number{};
  int weld_data_set_id{};
};

class WeldProgram {
 public:
  WeldProgram() = default;

  auto Id() const -> int;
  auto Name() const -> std::string;
  auto GrooveId() const -> int;
  auto Layers() const -> std::vector<WeldProgramLayer>;

  void SetId(int);
  void SetName(std::string);
  void SetGrooveId(int);
  void SetLayers(std::vector<WeldProgramLayer>);

  void AddLayer(WeldProgramLayer const&);
  auto NumLayers() const -> uint;
  auto IsValid() const -> bool;
  auto ToString() const -> std::string;
  auto ToJson() const -> nlohmann::json;
  static auto FromJson(const nlohmann::json&) -> std::optional<WeldProgram>;

  static void CreateTable(SQLite::Database*);
  static auto StoreFn() -> std::function<bool(SQLite::Database*, const WeldProgram&)>;
  static auto RemoveFn() -> std::function<bool(SQLite::Database*, int)>;
  static auto GetAllFn() -> std::function<std::vector<WeldProgram>(SQLite::Database*)>;

 private:
  static auto ReadLayers(SQLite::Database*, int) -> std::vector<WeldProgramLayer>;

  int id_{};
  std::string name_;
  int groove_id_{};
  std::vector<WeldProgramLayer> layers_;
};

}  // namespace weld_control
