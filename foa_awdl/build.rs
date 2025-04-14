use esp_config::{generate_config, Validator, Value};

fn main() {
    generate_config(
        "foa_awdl",
        &[
            (
                "PEER_CACHE_SIZE",
                "Maximum number of peers in the cache.",
                Value::Integer(4),
                Some(Validator::PositiveInteger),
            ),
            (
                "NET_TX_BUFFERS",
                "The amount of TX buffers used for embassy_net_driver_channel.",
                Value::Integer(4),
                Some(Validator::PositiveInteger),
            ),
            (
                "NET_RX_BUFFERS",
                "The amount of RX buffers used for embassy_net_driver_channel.",
                Value::Integer(4),
                Some(Validator::PositiveInteger),
            ),
            (
                "EVENT_QUEUE_DEPTH",
                "The maximum number of events, that the event queue can hold",
                Value::Integer(4),
                Some(Validator::PositiveInteger),
            ),
        ],
        true,
    );
}
