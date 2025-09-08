# The code

1. [common](common/DOCS.md)
1. [components](components/DOCS.md)
1. [controller](controller/DOCS.md)
1. [slice_translator](slice_translator/DOCS.md)
1. [weld](weld/DOCS.md)

## Components overview

```plantuml
left to right direction

component "adaptio-core" {
  component "core" as core {
    component "data" as common::data {
    }

    component "file" as common::file {
    }

    component "fsm" as common::fsm {
    }

    component "joint_tracking" as common::joint_tracking {
    }

    component "scanner" as common::scanner {
    }
  }
}

component "common" as common {
  component "logging" as common::logging {
    component "quest" as common::logging::quest {
    }
  }
}

component "components" as components {
  component "configuration" as configuration {
  }

  component "kinematics" as components::kinematics {
  }

  component "scanner" as components::scanner {
    component "input" as components::scanner::input {
    }

    component "output" as components::scanner::output {
    }
  }

  component "recipe" as components::recipe {
    component "msg" as components::recipe::msg {
    }
  }

  configuration <-- components::recipe
  components::scanner <-- configuration
  components::scanner <-- components::recipe::msg
}

component "controller" as controller {

  component "systems" as controller::systems {
  }

  component "pn_driver" as controller::pn_driver {
  }

  component "simulation" as controller::simulation {
  }
}

component "slice_translator" as slice_translator {
  component "calibration" as calibration {
  }
}

component "weld" as weld {
  component "sequence" as weld::sequence {
    component "calibration" as weld::sequence::calibration {
    }

    component "tests" as weld::sequence::tests {
    }

    component "tracking" as weld::sequence::tracking {
    }
  }
}

component "zevs" as zevs {
  component "adapter" as zevs::adapter {
  }
}

components::kinematics <-- weld
components::kinematics <-- weld::sequence
components::kinematics <-- weld::sequence::calibration
components::kinematics <-- weld::sequence::tests
components::kinematics <-- weld::sequence::tracking
components::scanner <-- weld
components::scanner <-- weld::sequence

common::data <-- configuration
common::file <-- configuration
common::fsm <-- weld::sequence
common::joint_tracking <- components::scanner
common::scanner <-- configuration
common::scanner <-- components::scanner

controller <-- configuration

slice_translator <-- configuration
calibration <-- weld
calibration <-- weld::sequence

zevs::EventLoop <-- components::scanner
zevs::EventLoop <-- controller
zevs::Socket <-- components::kinematics
zevs::Socket <-- components::recipe
zevs::Socket <-- components::scanner
zevs::Socket <-- controller
zevs::Socket <-- weld
zevs::Timer <-- components::scanner
zevs::Timer <-- controller

```
