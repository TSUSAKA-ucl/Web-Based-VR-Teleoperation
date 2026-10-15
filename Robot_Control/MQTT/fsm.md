```mermaid
stateDiagram-v2
    [*] --> init
    
    init : init
    init : entry / create mqtt.Client
    
    init --> starting

    starting : starting
    starting : entry / callback登録,connect,loop_start

    starting --> connected : on_connect

    connected : registration
    connected : entry / publish mgr/register

    connected --> registered : ((recv registered?))

    registered : registered
    registered : (( entry / recv key))
    registered : do / waiting for acquired by USER

    registered --> acquired : acquired<br>(recv dev/[robot-id])

    acquired : acquired
    acquired : do / pub joints(REAL)
    acquired : do / recv joints(VR)
    acquired : do / polling  diff b/w REAL&VR

    acquired --> running : small enough diff<br> of VR joints and mine

    running : running
    running : do / move
    
    running --> stopping : acquired<br>(recv dev/[robot-id])
    running --> stopping : too much diff

    stopping : soft stopping
    stopping : do / soft stop

    stopping --> acquired : zero velocites

    running --> stopping2 : ボタン, SIGINT

    stopping2 : soft stopping E
    stopping2 : do / soft stop

    stopping2 --> exiting : zero velocities

    exiting : exiting
    exiting : exit/ pub mgr/unregister

    exiting --> [*] : ((recv unregistered?))
```
