use esp_config::{generate_config, ConfigOption, Validator, Value};

fn main() {
    generate_config(
        "foa",
        &[
            ConfigOption::new(
                "rx_buffer_count",
                "Amount of RX buffers",
                Value::Integer(10),
            )
            .constraint(Validator::PositiveInteger).stable("0.1.0"),
            ConfigOption::new(
                "rx_queue_len",
                "Amount of frames the RX queue of an interface can hold",
                Value::Integer(2),
            )
            .constraint(Validator::PositiveInteger).stable("0.1.0"),
            ConfigOption::new(
                "tx_buffer_count",
                "Amount of TX buffers",
                Value::Integer(10),
            )
            .constraint(Validator::PositiveInteger).stable("0.1.0"),
        ],
        false,
        true,
    );
}
