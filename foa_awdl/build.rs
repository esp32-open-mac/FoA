use esp_config::{generate_config, Validator, Value};

fn main() {
    generate_config(
        "foa_awdl",
        &[(
            "PEER_CACHE_SIZE",
            "Maximum number of peers in the cache.",
            Value::Integer(4),
            Some(Validator::PositiveInteger),
        )],
        true,
    );
}
