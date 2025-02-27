# Ferris on Air
Ferris on Air (FoA) is an open source 802.11 stack for the ESP32 written in async rust, with the work of the [esp32-open-mac](https://esp32-open-mac.be/) project. The stack is intended to be used with [embassy](https://embassy.dev/) and is still in very early stages of development. We do not claim to be Wi-Fi certified, but implement the features specified by IEEE 802.11 to our best knowledge.
 ## Design
FoA uses a modular design, which allows implementing different interfaces, like STA, AP, etc. through a set of traits specified in [interface]. Currently only a very basic STA interface is implemented, which allows connecting to an open network. See [sta].
## Structure
Everything shared by all interfaces is in `foa`, this includes the LMAC, TX buffer management, interface traits and background task.
`foa_sta` contains a rudimentary implementation of a station interface.
`examples` contain a set of examples showing how to use different parts of the stack.
## Usage
For a concrete usage example, see `examples/src/bin/test.rs`.
## License
This project is licensed under Apache 2.0 or MIT at your option.
