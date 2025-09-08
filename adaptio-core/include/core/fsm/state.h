#pragma once

#include <boost/outcome.hpp>
#include <chrono>
#include <expected>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace core::fsm {

class State;
class Transition;

/**
 * An action lambda used in Enter, Tick, Exit and Timeout.
 * @param state A pointer to the state executing this lambda.
 */
using Action = std::function<void(const core::fsm::State* state)>;

// TODO(ADPT-111): Convert to constexpr variadic template
// NOLINTNEXTLINE
#define ACTION(...) [__VA_ARGS__]([[maybe_unused]] const core::fsm::State* state) -> void

/**
 * A condition lambda used when determining if the state machine should switch to the next state.
 * @param state A pointer to the state executing this lambda.
 */
using Condition = std::function<bool(const core::fsm::State* state)>;

// TODO(ADPT-111): Convert to constexpr variadic template
// NOLINTNEXTLINE
#define CONDITION(...) [__VA_ARGS__]([[maybe_unused]] const core::fsm::State* state) -> bool

class State;
using StatePtr = std::unique_ptr<State>;

class Transition;
using TransitionPtr = std::unique_ptr<Transition>;

/**
 * Class representing a state in a FSM
 */
class State {
  friend class StateBuilder;
  // Lets us use std::make_unique with private constructor.
  friend auto std::make_unique<State>(const std::string&) -> StatePtr;

 public:
  State()                           = delete;
  State(State&)                     = delete;
  auto operator=(State&) -> State&  = delete;
  State(State&&)                    = delete;
  auto operator=(State&&) -> State& = delete;

  ~State() = default;

  /**
   * Method executed once when entering the state.
   */
  void Enter();

  /**
   * Method executed every tick of the state machine driver owning this state.
   */
  void Tick();

  /**
   * Method executed once when exiting the state.
   */
  void Exit();

  /**
   * Adds an action that executes when entering this state.
   * Actions will be executed in the order added with this function.
   *
   * @param action The action
   */
  void AddOnEnter(const Action& action);

  /**
   * Adds an action that executes when ticking this state.
   * Actions will be executed in the order added with this function.
   *
   * @param action The action
   */
  void AddOnTick(const Action& action);

  /**
   * Adds an action that executes when exiting this state.
   * Actions will be executed in the order added with this function.
   *
   * @param action The action
   */
  void AddOnExit(const Action& action);

  /**
   * Sets a time limit to this state.
   *
   * @param time_limit The time limit.
   */
  void SetTimeout(std::chrono::duration<double> time_limit);

  /**
   * Adds an action that executes when this state times out.
   * NOTE: time limit must also be set with SetTimout(...)
   *
   * @param action The action
   */
  void AddOnTimeout(const Action& action);

  /**
   * Adds a transition to a child state of this state, _to this state_.
   * The super transitions are used to manipulate the active child state in order to facilitate more global transitions.
   */
  void AddSuperTransition(TransitionPtr transition);

  /**
   * Adds a state to the list of states that this state can branch to.
   * The transitions will be evaluated in the same order as they are added.
   *
   * @param transition The transition
   */
  void AddTransition(TransitionPtr transition);

  /**
   * Check if any super transitions should be executed for this state.
   *
   * @return The next child state of the current state, std::nullopt if not.
   */
  [[nodiscard]] auto UpdateSuperTransitions() const -> std::optional<State*>;

  /**
   * Check if any transitions should be executed for this state.
   *
   * @return The next state if this state should be exited, std::nullopt if not.
   */
  [[nodiscard]] auto UpdateTransitions() -> std::optional<State*>;

  /**
   * @return The pretty name of this state.
   */
  [[nodiscard]] auto GetName() const -> std::string;

  /**
   * @return Gets the name of this state and all substates breadcrumbs style.
   */
  [[nodiscard]] auto GetCompleteName() const -> std::string;

  /**
   * @return true if this state has timed out.
   */
  [[nodiscard]] auto IsTimedOut() const -> bool;

  /**
   * @return The time spent in this state.
   */
  [[nodiscard]] auto GetTimeSpent() const -> std::chrono::duration<double>;

  /**
   * @return True if this state is at end state.
   */
  auto AtEndState() -> bool;

  /**
   * Sets a data pointer in this state that can be used to pass information to the state.
   * The data pointer will be saved as a void pointer in the object and can later be accessed with GetData in e.g. an
   * action. NOTE: This can easily result in pointers to invalid data resulting in segfaults. Care must be taking when
   * using this functionality.
   */
  void SetData(std::optional<void*> data);

  /**
   * Optionally gets a raw pointer to the data contained in this state.
   * NOTE: Care must be taken when using this to avoid data corruption or invalid pointers.
   *
   * @return An std::optional that maybe contains a raw pointer to data set with SetData(...)
   */
  [[nodiscard]] auto GetData() const -> std::optional<void*>;

 private:
  std::string name_;
  std::vector<TransitionPtr> super_transitions_;
  std::vector<TransitionPtr> transitions_;
  std::vector<StatePtr> states_;
  std::optional<State*> current_state_ = std::nullopt;
  std::optional<State*> end_state_     = std::nullopt;
  std::vector<Action> on_enter_;
  std::vector<Action> on_tick_;
  std::vector<Action> on_exit_;
  std::vector<Action> on_timeout_;
  std::optional<std::chrono::duration<double>> time_limit_;
  std::chrono::time_point<std::chrono::high_resolution_clock> enter_time_;
  std::chrono::duration<double> time_spent_;
  std::optional<void*> data_;

  /**
   * Creates a state with a given name. The name is used for debugging/logging purposes.
   */
  explicit State(const std::string& name);

  /**
   * Adds a child state to this state. The child states will work as a FSM in
   * this state. The first state added will be set as current.
   *
   * NOTE: All states used by this superstate must be added to transfer ownership.
   */
  void AddState(StatePtr state);

  /**
   * @return A pointer to the current state executed in this state
   */
  [[nodiscard]] auto GetState() const -> std::optional<State*>;

  /**
   * Set the currently executed state in this state
   *
   * @param new_state A raw pointer to the new state
   */
  void SetState(State* new_state);

  /**
   * Sets the end state. If this is set to a valid pointer, the sub sequence
   * must be in this state before we can progress to the next state.
   *
   * @param end_state The end state, to disable pass std::nullopt
   */
  void SetEndState(std::optional<State*> end_state);

  /**
   * Gets the current end state.
   *
   * @return An std::optional containing a pointer to the end state, if there is one.
   */
  auto GetEndState() -> std::optional<State*>;

  /**
   * Resets the currently executed state to be the first state added to this
   * state.
   */
  void Reset();

  /**
   * Fetches the next state in the chain.
   *
   * @return A pointer to the next state that meets the conditions.
   * std::nullopt is returned if no conditions are met.
   */
  [[nodiscard]] auto GetNextState() const -> std::optional<State*>;
};

enum class StateBuilderErrorCode : uint32_t {
  NO_ERROR                           = 0,
  INVALID_POWER_SOURCE_CONFIGURATION = 10,
  EMPTY_STATE                        = 20,
};

// NOLINTNEXTLINE(*-identifier-naming)
[[maybe_unused]] auto make_error_code(StateBuilderErrorCode) -> std::error_code;

class StateBuilder {
 public:
  /**
   * Starts construction of a new state.
   * @param state_name The name of the new state.
   */
  explicit StateBuilder(const std::string& state_name);

  StateBuilder(StateBuilder&)                     = delete;
  auto operator=(StateBuilder&) -> StateBuilder&  = delete;
  StateBuilder(StateBuilder&&)                    = delete;
  auto operator=(StateBuilder&&) -> StateBuilder& = delete;

  ~StateBuilder() = default;

  /**
   * Adds an entry action to the state.
   */
  auto OnEnter(const Action& action) -> StateBuilder&;

  /**
   * Adds a tick action to the state.
   */
  auto OnTick(const Action& action) -> StateBuilder&;

  /**
   * Adds an exit action to the state.
   */
  auto OnExit(const Action& action) -> StateBuilder&;

  /**
   * Sets a time limit of this state.
   */
  auto SetTimeout(std::chrono::duration<double> time_limit) -> StateBuilder&;

  /**
   * Adds a timeout action to the state.
   */
  auto OnTimeout(const Action& action) -> StateBuilder&;

  /**
   * Allows this state to be empty (i.e. no actions, no transitions, no sub states).
   */
  auto AllowEmpty() -> StateBuilder&;

  /**
   * Adds a transition to a child state of this state, _to this state_.
   * The super transitions are used to manipulate the active child state in order to facilitate more global transitions.
   */
  auto AddSuperTransition(TransitionPtr transition) -> StateBuilder&;

  /**
   * Adds a transition from this state to another state on the same level in the state hierarchy.
   */
  auto AddTransition(TransitionPtr transition) -> StateBuilder&;

  auto AddState(StatePtr state) -> StateBuilder&;
  auto SetEndState(State* state) -> StateBuilder&;

  auto Finalize() -> boost::outcome_v2::result<StatePtr>;

 private:
  StatePtr state_;
  bool allow_empty_;
};

enum TransitionQualifier {
  ON_END,
  ALWAYS,
};

class Transition {
  friend class TransitionBuilder;
  friend class StateBuilder;
  friend class State;
  // Lets us use std::make_unique with private constructor.
  friend auto std::make_unique<Transition>(const std::string&) -> TransitionPtr;

 public:
  Transition()                                = delete;
  Transition(Transition&)                     = delete;
  auto operator=(Transition&) -> Transition&  = delete;
  Transition(Transition&&)                    = delete;
  auto operator=(Transition&&) -> Transition& = delete;

  ~Transition() = default;

  /**
   * Evaluates this transition.
   *
   * @return True if the condition is fulfilled.
   */
  [[nodiscard]] auto Evaluate() const -> bool;

  /**
   * Fetches the state this transition leads to.
   *
   * @return A raw pointer to the next state.
   */
  [[nodiscard]] auto GetTarget() const -> State*;

  /**
   * @return The name of this transition.
   */
  [[nodiscard]] auto GetName() const -> std::string;

 private:
  std::string name_;
  Condition condition_;
  TransitionQualifier qualifier_;
  State* target_{};
  State* parent_{};

  explicit Transition(std::string name);

  void SetCondition(const Condition& condition, TransitionQualifier qualifier);
  void SetTarget(State* state);
  void SetParent(core::fsm::State* state);
};

enum class TransitionBuilderErrorCode : uint32_t {
  NO_ERROR      = 0,
  NO_TARGET_SET = 10,
};

// NOLINTNEXTLINE(*-identifier-naming)
[[maybe_unused]] auto make_error_code(TransitionBuilderErrorCode) -> std::error_code;

class TransitionBuilder {
 public:
  explicit TransitionBuilder(const std::string& name);

  TransitionBuilder(TransitionBuilder&)                     = delete;
  auto operator=(TransitionBuilder&) -> TransitionBuilder&  = delete;
  TransitionBuilder(TransitionBuilder&&)                    = delete;
  auto operator=(TransitionBuilder&&) -> TransitionBuilder& = delete;

  ~TransitionBuilder() = default;

  auto SetCondition(const Condition& condition, TransitionQualifier qualifier = TransitionQualifier::ON_END)
      -> TransitionBuilder&;
  auto SetTarget(State* state) -> TransitionBuilder&;

  auto Finalize() -> boost::outcome_v2::result<TransitionPtr>;

 private:
  TransitionPtr transition_;
};

}  // namespace core::fsm

namespace std {
template <>
struct is_error_code_enum<core::fsm::StateBuilderErrorCode> : true_type {};
}  // namespace std

namespace std {
template <>
struct is_error_code_enum<core::fsm::TransitionBuilderErrorCode> : true_type {};
}  // namespace std
