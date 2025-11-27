#include "weld_control/src/weld_control_metrics_impl.h"

#include "weld_control/src/confident_slice_buffer.h"

namespace weld_control {

WeldControlMetricsImpl::WeldControlMetricsImpl(prometheus::Registry* registry) : registry_(registry) {};

void WeldControlMetricsImpl::ObserveLatency(int count) {
  if (nullptr == abw_latency_lpcs_seconds_) {
    LOG_ERROR("No Abw latency lpcs seconds available!");
    return;
  }

  abw_latency_lpcs_seconds_->Observe(count);
}

void WeldControlMetricsImpl::SetGroove(const std::optional<common::Groove>& g) {
  auto const top_width    = g->TopWidth();
  auto const bottom_width = g->BottomWidth();
  auto const area         = g->Area();
  auto const top_height_diff =
      g->operator[](common::ABW_UPPER_LEFT).vertical - g->operator[](common::ABW_UPPER_RIGHT).vertical;

  if (groove_.top_width_mm != nullptr) {
    groove_.top_width_mm->Set(top_width);
  }
  if (groove_.bottom_width_mm != nullptr) {
    groove_.bottom_width_mm->Set(bottom_width);
  }
  if (groove_.area_mm != nullptr) {
    groove_.area_mm->Set(area);
  }
  if (groove_.top_height_diff_mm != nullptr) {
    groove_.top_height_diff_mm->Set(top_height_diff);
  }
}

void WeldControlMetricsImpl::Setup(ConfidentSliceBuffer& confident_slice_buffer) {
  {
    auto& counter = prometheus::BuildCounter()
                        .Name("weld_control_input_slice_confidences_total")
                        .Help("Confidence level for scanner input slices.")
                        .Register(*registry_);

    slice_confidence_.emplace(lpcs::SliceConfidence::NO, &counter.Add({
                                                             {"confidence", "no"}
    }));
    slice_confidence_.emplace(lpcs::SliceConfidence::LOW, &counter.Add({
                                                              {"confidence", "low"}
    }));
    slice_confidence_.emplace(lpcs::SliceConfidence::MEDIUM, &counter.Add({
                                                                 {"confidence", "medium"}
    }));
    slice_confidence_.emplace(lpcs::SliceConfidence::HIGH, &counter.Add({
                                                               {"confidence", "high"}
    }));
  }

  {
    auto& counter = prometheus::BuildCounter()
                        .Name("weld_control_confident_slice_results_total")
                        .Help("Confident slice handling results.")
                        .Register(*registry_);

    confident_slice_.ok                 = &counter.Add({
        {"result", "ok"}
    });
    confident_slice_.no_data            = &counter.Add({
        {"result", "no_data"}
    });
    confident_slice_.translation_failed = &counter.Add({
        {"result", "translation_failed"}
    });
  }

  {
    confident_slice_buffer_fill_ratio_ = &prometheus::BuildGauge()
                                              .Name("weld_control_confident_slice_buffer_fill_ratio")
                                              .Help("Confident slice buffer ratio of number of filled slots.")
                                              .Register(*registry_)
                                              .Add({});
  }

  confident_slice_buffer_fill_ratio_->Set(static_cast<double>(confident_slice_buffer.FilledSlots()) /
                                          static_cast<double>(confident_slice_buffer.Slots()));

  {
    const std::vector<double> buckets = {
        0.030,  // 30 ms
        0.040,  // 40 ms
        0.050,  // 50 ms
        0.060,  // 60 ms
        0.070,  // 70 ms
        0.080,  // 80 ms
        0.090,  // 90 ms
        0.100,  // 100 ms
        0.150,  // 150 ms
        0.200,  // 200 ms
        0.300,  // 300 ms
        0.400,  // 400 ms
    };

    auto& histogram = prometheus::BuildHistogram()
                          .Name("adaptio_abw_latency_lpcs_seconds")
                          .Help("ABW data latency from image taken")
                          .Register(*registry_);
    abw_latency_lpcs_seconds_ = &histogram.Add({}, buckets);
  }

  {
    groove_.top_width_mm = &prometheus::BuildGauge()
                                .Name("weld_control_groove_top_width")
                                .Help("Joint Top Width.")
                                .Register(*registry_)
                                .Add({});
    groove_.bottom_width_mm = &prometheus::BuildGauge()
                                   .Name("weld_control_groove_bottom_width")
                                   .Help("Joint Bottom Width.")
                                   .Register(*registry_)
                                   .Add({});
    groove_.area_mm =
        &prometheus::BuildGauge().Name("weld_control_groove_area").Help("Joint area.").Register(*registry_).Add({});
    groove_.top_height_diff_mm = &prometheus::BuildGauge()
                                      .Name("weld_control_groove_top_height_diff")
                                      .Help("Joint Top Height Diff.")
                                      .Register(*registry_)
                                      .Add({});
  }
}

void WeldControlMetricsImpl::SetConfidentSliceBuffer(ConfidentSliceBuffer& confident_slice_buffer) {
  confident_slice_buffer_fill_ratio_->Set(static_cast<double>(confident_slice_buffer.FilledSlots()) /
                                          static_cast<double>(confident_slice_buffer.Slots()));
}

void WeldControlMetricsImpl::ResetGroove() {
  groove_.top_width_mm->Set(0.0);
  groove_.bottom_width_mm->Set(0.0);
  groove_.top_height_diff_mm->Set(0.0);
}

void WeldControlMetricsImpl::IncConfidentSliceNoData() { confident_slice_.no_data->Increment(); }

void WeldControlMetricsImpl::IncConfidentSliceOk() { confident_slice_.ok->Increment(); }

void WeldControlMetricsImpl::IncConfidentSliceTranslationFailed() { confident_slice_.translation_failed->Increment(); }

void WeldControlMetricsImpl::IncSliceConfidence(lpcs::SliceConfidence& confidence) {
  slice_confidence_[confidence]->Increment();
}
}  // namespace weld_control
