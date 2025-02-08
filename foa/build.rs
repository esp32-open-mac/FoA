use esp_config::{generate_config, Validator, Value};

fn main() {
    generate_config(
        "foa",
        &[
            (
                "rx_buffer_size",
                "Size of an RX buffer",
                Value::Integer(1600),
                Some(Validator::PositiveInteger),
            ),
            (
                "rx_buffer_count",
                "Amount of RX buffers",
                Value::Integer(10),
                Some(Validator::PositiveInteger),
            ),
            (
                "rx_queue_len",
                "Amount of frames the RX queue of an interface can hold",
                Value::Integer(2),
                Some(Validator::PositiveInteger),
            ),
            (
                "tx_buffer_count",
                "Amount of TX buffers",
                Value::Integer(10),
                Some(Validator::PositiveInteger),
            ),
            (
                "tx_buffer_size",
                "Size of a TX buffer",
                Value::Integer(1600),
                Some(Validator::PositiveInteger),
            ),
        ],
        true,
    );
}
