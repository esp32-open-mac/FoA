use esp_config::{Validator, Value, generate_config};

fn main() {
    generate_config(
        "foa_mesh",
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
