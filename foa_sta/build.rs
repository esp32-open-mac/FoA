use esp_config::{generate_config, Validator, Value};

fn main() {
    generate_config(
        "foa_sta",
        &[
            (
                "RX_QUEUE_DEPTH",
                "The depth of the user and background RX queues.",
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
        ],
        true,
    );
}
