

#include "bead.h"

#include "arc2d.h"
#include "intersection-point.h"

namespace deposition_simulator {

Bead::Bead() = default;
Bead::Bead(const Arc2d &arc, IntersectionPoint &start, IntersectionPoint &end) : arc_(arc), start_(start), end_(end) {}

auto Bead::GetStart() const -> IntersectionPoint { return this->start_; }

auto Bead::GetEnd() const -> IntersectionPoint { return this->end_; }

auto Bead::GetArc() const -> Arc2d { return this->arc_; }
}  // namespace deposition_simulator
