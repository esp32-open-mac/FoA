use esp_config::{generate_config, ConfigOption, Validator, Value};

fn main() {
    generate_config(
        "foa_awdl",
        &[
            ConfigOption::new(
                "PEER_CACHE_SIZE",
                "Maximum number of peers in the cache.",
                Value::Integer(4),
            )
            .constraint(Validator::PositiveInteger),
            ConfigOption::new(
                "NET_TX_BUFFERS",
                "The amount of TX buffers used for embassy_net_driver_channel.",
                Value::Integer(4),
            )
            .constraint(Validator::PositiveInteger),
            ConfigOption::new(
                "NET_RX_BUFFERS",
                "The amount of RX buffers used for embassy_net_driver_channel.",
                Value::Integer(4),
            )
            .constraint(Validator::PositiveInteger),
            ConfigOption::new(
                "EVENT_QUEUE_DEPTH",
                "The maximum number of events, that the event queue can hold",
                Value::Integer(4),
            )
            .constraint(Validator::PositiveInteger),
        ],
        false,
        true,
    );
}
