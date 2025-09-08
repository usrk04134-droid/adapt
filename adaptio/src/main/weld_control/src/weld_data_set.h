#pragma once

#include <SQLiteCpp/Database.h>

#include <functional>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace weld_control {

class WeldDataSet {
 public:
  WeldDataSet() = default;

  auto Id() const -> int;
  auto Name() const -> std::string;
  auto Ws1WppId() const -> int;
  auto Ws2WppId() const -> int;

  void SetId(int);
  void SetName(std::string);
  void SetWs1WppId(int);
  void SetWs2WppId(int);

  auto IsValid() const -> bool;
  auto ToString() const -> std::string;
  auto ToJson() const -> nlohmann::json;
  static auto FromJson(const nlohmann::json&) -> std::optional<WeldDataSet>;

  static void CreateTable(SQLite::Database*);
  static auto StoreFn() -> std::function<bool(SQLite::Database*, const WeldDataSet&)>;
  static auto RemoveFn() -> std::function<bool(SQLite::Database*, int)>;
  static auto GetAllFn() -> std::function<std::vector<WeldDataSet>(SQLite::Database*)>;

 private:
  int id_{};
  std::string name_;
  int ws1_wpp_id_{};
  int ws2_wpp_id_{};
};

}  // namespace weld_control
