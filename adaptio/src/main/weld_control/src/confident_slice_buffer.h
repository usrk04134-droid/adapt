#pragma once

#include <SQLiteCpp/SQLiteCpp.h>

#include <memory>
#include <nlohmann/json.hpp>
#include <numbers>

#include "common/logging/application_log.h"
#include "common/storage/slot_buffer_storage.h"
#include "common/groove/groove.h"

namespace {}  // namespace

namespace weld_control {

struct ConfidentSliceData {
  double edge_position;
  common::groove::Groove groove;

  auto ToJson() const -> nlohmann::json {
    return nlohmann::json{
        {"edgePosition", edge_position  },
        {"groove",       groove.ToJson()}
    };
  }

  static auto FromJson(const nlohmann::json& json_obj) -> std::optional<ConfidentSliceData> {
    try {
      auto groove_opt = common::groove::Groove::FromJson(json_obj.at("groove"));
      if (!groove_opt.has_value()) {
        return std::nullopt;
      }

      return ConfidentSliceData{.edge_position = json_obj.at("edgePosition").get<double>(),
                                .groove        = groove_opt.value()};
    } catch (const nlohmann::json::exception& e) {
      LOG_ERROR("Failed to parse ConfidentSliceData from JSON: {}", e.what());
      return std::nullopt;
    }
  }
};

class ConfidentSliceBuffer {
 public:
  explicit ConfidentSliceBuffer(SQLite::Database* db)
      : storage_(std::make_unique<common::storage::SlotBufferStorage<ConfidentSliceData>>(db, "confident_slice")) {}

  auto Init(double radius, double resolution) -> void {
    auto const slots = static_cast<size_t>(2 * std::numbers::pi * radius / resolution);
    storage_->Init(slots, 2 * std::numbers::pi);
  }

  auto Store(double pos, ConfidentSliceData const& value) -> void { storage_->Store(pos, value); }

  auto Get(double pos) const -> std::optional<std::pair<double, ConfidentSliceData>> { return storage_->Get(pos); }

  auto Clear() -> void { storage_->Clear(); }

  auto Empty() const -> bool { return storage_->Empty(); }

  auto Filled() const -> bool { return storage_->Filled(); }

  auto FilledSlots() const -> size_t { return storage_->FilledSlots(); }

  auto Slots() const -> size_t { return storage_->Slots(); }

  auto FillRatio() const -> double { return storage_->FillRatio(); }

  auto Available() const -> bool { return storage_->Available(); }

 private:
  std::unique_ptr<common::storage::SlotBufferStorage<ConfidentSliceData>> storage_;
};

}  // namespace weld_control
