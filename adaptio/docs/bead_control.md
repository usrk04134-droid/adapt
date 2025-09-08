# Bead Control

```plantuml
hide empty description
[*] -> Update : BeadControlInput
state Update {
IDLE :
REPOSITIONING : When entering this state:\nDecide where to place the next bead \nand move to new bead position.\n\nWhen in this state:\nUpdate output to keep constant\nvelocity or desired bead switch angle.

STEADY : Welding bead.

OVERLAPPING : Welding overlap.

IDLE --> REPOSITIONING

REPOSITIONING -> STEADY: finished
STEADY --> OVERLAPPING: finished
OVERLAPPING --> REPOSITIONING: finished
}
Update -> [*] : BeadControlOutput
```
