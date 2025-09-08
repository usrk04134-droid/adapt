# Weld Control status

```plantuml
participant WebHMI as FE
box #LightBlue
participant "Adaptio application" as A
end box

note over FE
JSON
{
  name: "GetWeldControlStatus",
}
end note
FE->A
note over A
JSON
{
  name: "GetWeldControlStatusRsp",
  payload = {
        weldControlState: "abp",
        beadControlState: "steady",
        progress: 0.45,
        beadNumber: 3,
        layerNumber: 2,
  }
}
end note
FE<--A
```

# Weld data set

## Create

```plantuml
participant WebHMI as FE
box #LightBlue
participant "Adaptio application" as A
database Database as DB
end box

note over FE
JSON
{
  name: "AddWeldDataSet",
  payload: {
    name: "data-set",
    type: "root",
    weldSpeed: 10,
    ...
  }
}
end note
FE->A
A->DB
A<--DB
note over A
JSON
{
  name: "AddWeldDataSet",
  payload: {
    result: "ok"
  }
}
end note
FE<--A
```

## Update

```plantuml
participant WebHMI as FE
box #LightBlue
participant "Adaptio application" as A
database Database as DB
end box

note over FE
JSON
{
  name: "UpdateWeldDataSet",
  payload: {
    id: 1,
    name: "new-data-set",
    type: "root",
    weldSpeed: 15,
    ...
  }
}
end note
FE->A
A->DB
A<--DB
note over A
JSON
{
  name: "UpdateWeldDataSet",
  payload: {
    result: "ok"
  }
}
end note
FE<--A
```

## Remove

```plantuml
participant WebHMI as FE
box #LightBlue
participant "Adaptio application" as A
database Database as DB
end box

note over FE
JSON
{
  name: "RemoveWeldDataSet",
  payload: {
    id: 1
  }
}
end note
FE->A
A->DB
A<--DB
note over A
JSON
{
  name: "RemoveWeldDataSet",
  payload: {
    result: "ok"
  }
}
end note
FE<--A
```

## Get

```plantuml
participant WebHMI as FE
box #LightBlue
participant "Adaptio application" as A
database Database as DB
end box

note over FE
JSON
{
  name: "GetWeldDataSets",
}
end note
FE->A
A->DB
A<--DB
note over A
JSON
{
  name: "GetWeldDataSets",
  payload = {
    [
      {
        id: 1,
        name: "data-set-root",
        type: "root",
        weldSpeed: 10,
        ...
      },
      {
        id: 1,
        name: "data-fill1",
        type: "fill",
        weldSpeed: 15,
        ...
      },
    ]
  }
}
end note
FE<--A
```

# Weld program

## Store

```plantuml
participant WebHMI as FE
box #LightBlue
participant "Adaptio application" as A
database Database as DB
end box

note over FE
JSON
{
  name: "StoreWeldProgram",
  payload: {
    layers: [
      {
        weldDataSetId: 1
        numberOfBeads: 1,
        beadPositions: [0.4],
        ...
      },
      {
        weldDataSetId: 2
        numberOfBeads: 2,
        beadPositions: [0.3, 0.7],
        ...
      },
      {
        weldDataSetId: 2
        numberOfBeads: 3,
        ...
      },
    ]
  }
}
end note
FE->A
A->DB
A<--DB
note over A
JSON
{
  name: "StoreWeldProgram",
  result: "ok"
}
end note
FE<-- A
```

## Get

```plantuml
participant WebHMI as FE
box #LightBlue
participant "Adaptio application" as A
database Database as DB
end box

note over FE
JSON
{
  name: "GetWeldProgram",
}
end note
FE->A
A->DB
A<--DB
note over A
JSON
{
  response: "GetWeldProgram",
  data: {
    layers: [
      {
        weldDataSetId: 1
        numberOfBeads: 1,
        beadPositions: [0.4],
        ...
      },
      {
        weldDataSetId: 2
        numberOfBeads: 2,
        beadPositions: [0.3, 0.7],
        ...
      },
      {
        weldDataSetId: 2
        numberOfBeads: 3,
        ...
      },
    ]
  }
}
end note
FE<--A
```
