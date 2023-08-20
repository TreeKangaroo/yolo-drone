Note all Tello drones are configured to have an IP of `192.168.10.1`

Each Tello drone must be connected to a separate wifi dongle

### Instructions:
(the 1 and 2 versions of the file can be used to simultaneously connect to different wingman drones; each will be ran in a different terminal window)

* optional: add `alias go=./go.py` to `~/.bashrc`, or replace `go` with `./go.py` for commands below
* configure parameters: `$ cp config.ini.dist config.ini` and [modify content accordingly](config.ini.dist)
* build docker image: `$ go build`
* start docker container: `$ go setup` (see `--help` for overridding parameters)
* turn on drone
* configure docker container to connect to drone's WiFi and setup UDP relays: `$ go connect`
* connect driver to drone using IP given from output of above command
* teardown: `$ go stop`
