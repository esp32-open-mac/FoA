use esp_config::{generate_config, Validator, Value};

macro_rules! gen_config_opt {
    ($name:expr, $description:expr, $default_value:expr, $validator:expr) => {
        esp_config::ConfigOption {
            name: $name.to_string(),
            description: $description.to_string(),
            default_value: $default_value,
            constraint: Some($validator),
            stability: esp_config::Stability::Stable("0.1.0".to_string()),
            active: true,
            display_hint: esp_config::DisplayHint::None
        }
    };
}

fn main() {
    generate_config(
        "foa",
        &[
            gen_config_opt!(
                "rx_buffer_count",
                "Amount of RX buffers",
                Value::Integer(10),
                Validator::PositiveInteger
            ),
            gen_config_opt!(
                "rx_queue_len",
                "Amount of frames the RX queue of an interface can hold",
                Value::Integer(2),
                Validator::PositiveInteger
            ),
            gen_config_opt!(
                "tx_buffer_count",
                "Amount of TX buffers",
                Value::Integer(10),
                Validator::PositiveInteger
            )
        ],
        false,
        true,
    );
}
