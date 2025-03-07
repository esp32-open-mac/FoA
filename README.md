# Ferris on Air
Ferris on Air (FoA) is an open source 802.11 stack for the ESP32 written in async rust, with the work of the [esp32-open-mac](https://esp32-open-mac.be/) project. The stack is intended to be used with [embassy](https://embassy.dev/) and is still in very early stages of development. We do not claim to be Wi-Fi certified, but implement the features specified by IEEE 802.11 to our best knowledge.
## Design
The main FoA crate acts as a multiplexer, that divides access to the hardware up into a number of virtual interfaces (VIF's). These can then be passed to interface implementations, like `foa_sta` or [`foa_dswifi`](https://github.com/mjwells2002/foa_dswifi). These interface implementations can coexist, enabling things like AP/STA operation in the future.
## Structure
The `foa` crate contains the LMAC, TX buffer management and RX ARC buffer management.
`foa_sta` contains a rudimentary implementation of a station interface.
`examples` contain a set of examples showing how to use different parts of the stack.
## Usage
For a concrete usage example, see `examples`. These examples can be run with `./run_example.sh <EXAMPLE_NAME> <CHIP> [SSID] [LOG_LEVEL]`.
## License
This project is licensed under Apache 2.0 or MIT at your option.
