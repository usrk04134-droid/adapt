# Diagram

```plantuml
title: Axis positioning

group **Set position**
   Controller <- Adaptio : AxisOutput(position, velocity, acceleration, jerk, execute=1)
end

group **Update status**
    Controller -> Adaptio: AxisInput(position, in_position=0/1)
end
```
