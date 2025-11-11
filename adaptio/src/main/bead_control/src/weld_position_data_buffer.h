#pragma once

#include <SQLiteCpp/SQLiteCpp.h>

#include <cassert>
#include <memory>
#include <nlohmann/json.hpp>
#include <numbers>

#include "common/groove/groove.h"
#include "common/logging/application_log.h"
#include "common/storage/slot_buffer_storage.h"

namespace {}  // namespace

namespace bead_control {

struct WeldPositionData {
  common::Groove groove;
  double left_bead_area{0.};
  double right_bead_area{0.};

  // DB not used at the moment for buffer
  auto ToJson() const -> nlohmann::json {
    assert(false);
    return nlohmann::json{};
  }
  static auto FromJson(const nlohmann::json& json_obj) -> std::optional<WeldPositionData> {
    assert(false);
    return std::nullopt;
  }
};

class WeldPositionDataBuffer {
 public:
  explicit WeldPositionDataBuffer(SQLite::Database* db)
      : storage_(std::make_unique<common::storage::SlotBufferStorage<WeldPositionData>>(db, "weld_position_data")) {}

  auto Init(double radius, double resolution) -> void {
    auto const slots = static_cast<size_t>(2 * std::numbers::pi * radius / resolution);
    storage_->Init(slots, 2 * std::numbers::pi);
  }

  auto Store(double pos, WeldPositionData const& value) -> void { storage_->Store(pos, value); }

  auto Get(double pos) const -> std::optional<std::pair<double, WeldPositionData>> { return storage_->Get(pos); }

  auto Clear() -> void { storage_->Clear(); }

  auto Empty() const -> bool { return storage_->Empty(); }

  auto Filled() const -> bool { return storage_->Filled(); }

  auto FilledSlots() const -> size_t { return storage_->FilledSlots(); }

  auto Slots() const -> size_t { return storage_->Slots(); }

  auto FillRatio() const -> double { return storage_->FillRatio(); }

  auto Available() const -> bool { return storage_->Available(); }

  // NOLINTBEGIN(readability-identifier-naming)
  using iterator = std::optional<std::pair<double, WeldPositionData>>*;
  auto begin() -> iterator { return storage_->begin(); };
  auto end() -> iterator { return storage_->end(); };
  // NOLINTEND(readability-identifier-naming)

 private:
  std::unique_ptr<common::storage::SlotBufferStorage<WeldPositionData>> storage_;
};

}  // namespace bead_control
