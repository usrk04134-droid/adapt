# FSM

## Rationale

The FSM constructs is a way to collect a sequence of events that must occur in a logical sequence.
It is constructed of a finite set of states connected together via transition functions.

A simple FSM describing the turnstile at ESABs Laxå site:

```plantuml
hide empty description
[*] -> Locked

Locked --> Unlocked
note on link
  Badge and correct pin presented
end note

Locked -> Locked
note on link
  Turnstile pushed
end note

Unlocked --> Locked
note on link
  Turnstile pushed
end note

Unlocked --> Locked
note on link
  Timeout
end note
```

In this application the FSM connects our various components together and executes functionality in a
controlled way while requiring certain checks to pass before transitioning further in the sequence.

## State

A state is a single control block that consists of several components:

- Entry action: One or more Actions that will be executed (in order added) **when entering a state**.
- Tick action: One or more Actions that will be executed (in order added) **on each Tick()**.
- Exit action: One or more Actions that will be executed (in order added) **when exiting a state**.

It also has a mechanism to control state [transition](#transition)s.

### Action

Defined as a std::function:

```c++
using Action = std::function<void(const core::fsm::State* state)>;
```

A helper macro exists to make it easier to define an Action:

```c++
#define ACTION(...) [__VA_ARGS__]([[maybe_unused]] const core::fsm::State* state) -> void
```

When creating a state, a new Action that sets a variable `bool *foo` to true can be declared as follows:

```c++
ACTION(foo) { *foo = true; }
```

### Transition

A transition is a construct used to determine if a state machine should switch between states.
It is defined as a std::function returning a bool:

```c++
using Condition = std::function<bool(const core::fsm::State* state)>;
```

A helper macro is defined to make it easier to define a Condition:

```c++
#define CONDITION(...) [__VA_ARGS__]([[maybe_unused]] const core::fsm::State* state) -> bool
```

A Condition that checks a variable `bool *foo` can be declared as follows:

```c++
CONDITION(foo) { return *foo; }
```

#### Condition

These are used in transitions to evaluate (together with the [Qualifier](#qualifier)) whether the transition should be executed.

#### Qualifier

The TransitionQualifier enum is an enum added on a [Condition](#condition) to determine if the condition should be evaluated or not,
it can currently have two values:

- ON_END: The transition will be executed if the condition returns true **and** the owning states current child state is equal to its end state (if set).
- ALWAYS: The transition will always be executed if the condition returns true, regardless of the value of current state.

#### Super transitions

Super transitions are normal transitions added to a special list in a state. These transitions are evaluated in a way that makes it possible to jump directly between child states.
This facilitates transitions that operates on a set of child states and makes it possible to abort a sequence regardless of which child state is currently executing.

## Examples

### Weld preamble

A small sequence that waits for a start command and sets read & active statuses.

```c++
using core::fsm::State;
using core::fsm::StateBuilder;
using core::fsm::TransitionBuilder;

auto end = StateBuilder("End").AllowEmpty().Finalize().value();

auto wait_for_start = StateBuilder("Wait for start command")
                        .OnEnter(ACTION(ready) { *ready = true; })
                        .OnExit(ACTION(ready, active) {
                          *active = true;
                          *ready  = false;
                        })
                        .AddTransition(TransitionBuilder("Start = true")
                                           .SetCondition(CONDITION(start) { return *start; })
                                           .SetTarget(end.get())
                                           .Finalize()
                                           .value())
                        .Finalize()
                        .value();

auto wait_for_no_start = StateBuilder("Wait until start command is low")
                           .AddTransition(TransitionBuilder("Start = false")
                                              .SetCondition(CONDITION(start) { return !*start; })
                                              .SetTarget(wait_for_start.get())
                                              .Finalize()
                                              .value())
                           .Finalize()
                           .value();

auto init = StateBuilder("Init")
              .OnEnter(ACTION(ready, active) {
                *ready  = false;
                *active = false;
              })
              .AddTransition(TransitionBuilder("Init done")
                                 .SetCondition(CONDITION() { return true; })
                                 .SetTarget(wait_for_no_start.get())
                                 .Finalize()
                                 .value())
              .Finalize()
              .value();

auto weld_preamble = StateBuilder("Weld preamble")
                       .SetEndState(end.get())
                       .AddState(std::move(init))
                       .AddState(std::move(wait_for_no_start))
                       .AddState(std::move(wait_for_start))
                       .AddState(std::move(end))
                       .Finalize()
                       .value();
```

This will result in a FSM as follows:

```plantuml
hide empty description
state "Weld preamble" as weld_preamble {
  state "Init" as init
  state "Wait until start command is low" as wait_for_no_start
  state "Wait for start command" as wait_for_start
  state "End" as end

  init: The initial state of this FSM
  end: This state is marked as the end of this FSM.\nThis is useful for transitions applied to "Weld preamble".

  note right of init
  OnEnter:
  ACTION(ready, active) { *ready  = false; *active = false; }
  end note

  note right of wait_for_start
  OnEnter:
  ACTION(ready) { *ready = true; }

  OnExit:
  ACTION(ready, active) { *active = true; *ready  = false; }
  end note

  [*] --> init
  note on link
    First state added is considered the "init" state and will be set as current state when State::Reset() is called.
  end note

  init --> wait_for_no_start
  note on link
    Transition { CONDITION() { return true; } // Since this is always true, we will execute the entry,
    tick and exit conditions once and then switch to next state.
  end note

  wait_for_no_start --> wait_for_start
  note on link
    Transition { CONDITION(start) { return !*start; } }
  end note

  wait_for_start --> end
  note on link
    Transition { CONDITION(start) { return *start; } }
  end note

  end --> [*]
}
```

### Interacting with the outside

To interact with the outside we can add pointers to other objects that does something useful, like moving an axis:

This FSM uses a pointer to a Kinematics-object that can manipulate axis, and also demonstrates the use of State::GetData()

```c++
using core::fsm::State;
using core::fsm::StateBuilder;
using core::fsm::TransitionBuilder;

auto end = StateBuilder("End").AllowEmpty().Finalize().value();

auto wait_until_stopped = StateBuilder("Wait until stopped")
                              .AddTransition(TransitionBuilder("Stopped")
                                                 .SetCondition(CONDITION(kinematics) {
                                                   return !kinematics->Busy() && !kinematics->InPosition();
                                                 })
                                                 .SetTarget(end.get())
                                                 .Finalize()
                                                 .value())
                              .Finalize()
                              .value();

auto wait_until_in_position = StateBuilder("Wait until in position")
                                  .OnExit(ACTION(kinematics) { kinematics->Stop(); })
                                  .AddTransition(TransitionBuilder("In position")
                                                     .SetCondition(CONDITION(kinematics) {
                                                       return kinematics->InPosition();
                                                     })
                                                     .SetTarget(wait_until_stopped.get())
                                                     .Finalize()
                                                     .value())
                                  .Finalize()
                                  .value();

auto wait_until_active = StateBuilder("Wait until active")
                             .OnEnter(ACTION(kinematics) { kinematics->Start(); })
                             .AddTransition(TransitionBuilder("Axis active")
                                                .SetCondition(CONDITION(kinematics) { return kinematics->Busy(); })
                                                .SetTarget(wait_until_in_position.get())
                                                .Finalize()
                                                .value())
                             .Finalize()
                             .value();

auto execute_move_command = StateBuilder("Execute move command")
                                .OnEnter(ACTION(kinematics) {
                                  if (state->GetData().has_value()) {
                                    auto* move_command =
                                        static_cast<components::kinematics::MoveAbsolute*>(state->GetData().value());
                                    kinematics->PushBack(*move_command);
                                  } else {
                                    LOG_ERROR("No move command set");
                                  }
                                })
                                .SetEndState(end.get())
                                .AddState(std::move(wait_until_active))
                                .AddState(std::move(wait_until_in_position))
                                .AddState(std::move(wait_until_stopped))
                                .AddState(std::move(end))
                                .Finalize()
                                .value();
```

FSM:

```plantuml
hide empty description
state "Execute move command" as execute_move_command {
  state "Wait until active" as wait_until_active
  state "Wait until in position" as wait_until_in_position
  state "Wait until stopped" as wait_until_stopped
  state "End" as end

  note right of wait_until_in_position
  OnExit:
    ACTION(kinematics) { kinematics->Stop(); }
  end note

  [*] --> wait_until_active

  wait_until_active --> wait_until_in_position
  note on link
    CONDITION(kinematics) { return kinematics->Busy(); }
  end note

  wait_until_in_position --> wait_until_stopped
  note on link
    CONDITION(kinematics) { return kinematics->InPosition(); }
  end note

  wait_until_stopped --> end
  note on link
    CONDITION(kinematics) { return !kinematics->Busy() && !kinematics->InPosition(); }
  end note

  end --> [*]
}

note right of execute_move_command
OnEnter:
  ACTION(kinematics) {
    if (state->GetData().has_value()) {
      auto* move_command =
          static_cast<components::kinematics::MoveAbsolute*>(state->GetData().value());
      kinematics->PushBack(*move_command);
    } else {
      LOG_ERROR("No move command set");
    }
  }
end note
```

### Composite FSM

Since "Weld preamble" and "Execute move commands" are really just states with a list of substates,
we can use them together in a FSM to execute the move command when start is true:

```c++
using core::fsm::State;
using core::fsm::StateBuilder;
using core::fsm::TransitionBuilder;
using components::kinematics::MoveAbsolute;

// The member variables ending with _ will be set/read from somewhere else, e.g. messages/callbacks from ZMQ or similar.
auto *start  = &start_;
auto *ready  = &ready_;
auto *active = &active_;

auto kinematics = kinematics_;

// It is important that target position still exists after the FSM is constructed,
// Here we use a member variable of this FSM class.
MoveAbsolute *target_position = &target_position_;

StatePtr weld_preamble = GetWeldPreamble(start, ready, active); // Helper function that returns a weld preamble FSM

// The kinematics pointer is passed to the constructor of this FSM class or set by other means.
StatePtr execute_move_command = GetMoveCommand(kinematics); // Helper function that returns a move command

execute_move_command->SetData(static_cast<void*>());

/*
 * Transition that will execute when CONDITION is true (always) AND when weld preamble reaches its end state
 * NOTE: TransitionQualifier::ON_END is the default value and can be omitted.
 */
weld_preamble->AddTransition(TransitionBuilder("Start is pressed")
                                .SetCondition(CONDITION() { return true; }, TransitionQualifier::ON_END)
                                .SetTarget(execute_move_command.get())
                                .Finalize()
                                .value());

// The root state of this FSM, to drive the entire FSM we call State::Tick() on this state only.
auto root_state = StateBuilder("Root state")
                      .AddState(std::move(weld_preamble))
                      .AddState(std::move(execute_move_command))
                      .Finalize()
                      .value();
```

FSM:

```plantuml
hide empty description
state "Root state" as root_state {
  state "Weld preamble" as weld_preamble
  state "Execute move command" as execute_move_command

  [*] --> weld_preamble
  note on link
    Initial state
  end note

  weld_preamble --> execute_move_command
  note on link
    // Given that this always returns true and TransitionQualifier::ON_END this
    // transition will be executed when "Weld preamble" reaches its end state.
    // i.e. the start boolean has been false followed by true.
    CONDITION() { return true; }
  end note

  execute_move_command --> [*]
}
```

Detailed FSM:

```plantuml
hide empty description
state "Root state" as root_state {
  state "Weld preamble" as weld_preamble {
    state "Init" as init
    state "Wait until start command is low" as wait_for_no_start
    state "Wait for start command" as wait_for_start
    state "End" as preamble_end

    preamble_end: This state is marked as the end of this FSM.\nThis is useful for transitions applied to "Weld preamble".

    note right of init
    OnEnter:
    ACTION(ready, active) { *ready  = false; *active = false; }
    end note

    note right of wait_for_start
    OnEnter:
    ACTION(ready) { *ready = true; }

    OnExit:
    ACTION(ready, active) { *active = true; *ready  = false; }
    end note

    [*] --> init

    init --> wait_for_no_start
    note on link
      Transition { CONDITION() { return true; }
    end note

    wait_for_no_start --> wait_for_start
    note on link
      Transition { CONDITION(start) { return !*start; } }
    end note

    wait_for_start --> preamble_end
    note on link
      Transition { CONDITION(start) { return *start; } }
    end note

    preamble_end --> [*]
  }

  state "Execute move command" as execute_move_command {
    state "Wait until active" as wait_until_active
    state "Wait until in position" as wait_until_in_position
    state "Wait until stopped" as wait_until_stopped
    state "End" as move_end

    note right of wait_until_in_position
    OnExit:
      ACTION(kinematics) { kinematics->Stop(); }
    end note

    [*] --> wait_until_active

    wait_until_active --> wait_until_in_position
    note on link
      CONDITION(kinematics) { return kinematics->Busy(); }
    end note

    wait_until_in_position --> wait_until_stopped
    note on link
      CONDITION(kinematics) { return kinematics->InPosition(); }
    end note

    wait_until_stopped --> move_end
    note on link
      CONDITION(kinematics) { return !kinematics->Busy() && !kinematics->InPosition(); }
    end note

    move_end --> [*]
  }

  note right of execute_move_command
  OnEnter:
    ACTION(kinematics) {
      if (state->GetData().has_value()) {
        auto* move_command =
            static_cast<components::kinematics::MoveAbsolute*>(state->GetData().value());
        kinematics->PushBack(*move_command);
      } else {
        LOG_ERROR("No move command set");
      }
    }
  end note

  [*] --> weld_preamble
  note on link
    Initial state
  end note

  weld_preamble --> execute_move_command
  note on link
    CONDITION() { return true; }
  end note

  execute_move_command --> [*]
}
```

### Super transitions

To facilitate a stop function we can build on the previous example ([Composite FSM](#composite-fsm)):

```c++
using core::fsm::State;
using core::fsm::StateBuilder;
using core::fsm::TransitionBuilder;
using components::kinematics::MoveAbsolute;

// The member variables ending with _ will be set/read from somewhere else, e.g. messages/callbacks from ZMQ or similar.
auto *start  = &start_;
auto *stop  = &stop_;
auto *ready  = &ready_;
auto *active = &active_;

auto kinematics = kinematics_;

// It is important that target position still exists after the FSM is constructed,
// Here we use a member variable of this FSM class.
MoveAbsolute *target_position = &target_position_;

StatePtr weld_preamble = GetWeldPreamble(start, ready, active); // Helper function that returns a weld preamble FSM

// The kinematics pointer is passed to the constructor of this FSM class or set by other means.
StatePtr execute_move_command = GetMoveCommand(kinematics); // Helper function that returns a move command

execute_move_command->SetData(static_cast<void*>());

/*
 * Transition that will execute when CONDITION is true (always) AND when weld preamble reaches its end state
 * NOTE: TransitionQualifier::ON_END is the default value and can be omitted.
 */
weld_preamble->AddTransition(TransitionBuilder("Start is pressed")
                                .SetCondition(CONDITION() { return true; }, TransitionQualifier::ON_END)
                                .SetTarget(execute_move_command.get())
                                .Finalize()
                                .value());

auto stop = StateBuilder("Stop")
                .OnEntry(ACTION(kinematics) {
                    kinematics_->Stop();
                })
                .Finalize()
                .value();

// The root state of this FSM, to drive the entire FSM we call State::Tick() on this state only.
auto root_state = StateBuilder("Root state")
                      .AddState(std::move(weld_preamble))
                      .AddState(std::move(execute_move_command))
                      // Super transitions are transitions that instead of moving from the state that owns the transition to another state,
                      // it is used to directly manipulate the child states of the owning state.
                      .AddSuperTransition(TransitionBuilder("Stop")
                                        .SetCondition(CONDITION(stop) { return *stop; })
                                        .SetTarget(stop.get())
                                        .Finalize()
                                        .value())
                      .AddState(std::move(stop));
                      .Finalize()
                      .value();
```

FSM:

```plantuml
hide empty description
state "Root state" as root_state {
  state "Not an actual container, used to visualize that\nwe can jump from any of these states to the Stop state." as container {
    state "Weld preamble" as weld_preamble
    state "Execute move command" as execute_move_command
  }
  state "Stop" as stop

  note right of stop
  OnEnter:
    ACTION(kinematics) { kinematics->Stop(); }
  end note

  [*] --> weld_preamble

  weld_preamble --> execute_move_command
  note on link
    CONDITION() { return true; }
  end note

  container --> stop
  note on link
    CONDITION(stop) { return *stop; }
  end note

  execute_move_command --> [*]
}
```
