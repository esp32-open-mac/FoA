use esp_config::{generate_config, ConfigOption, Validator, Value};

fn main() {
    generate_config(
        "foa_sta",
        &[
            ConfigOption::new(
                "RX_QUEUE_DEPTH",
                "The depth of the user and background RX queues.",
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
        ],
        false,
        true,
    );
}
