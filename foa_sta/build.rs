use esp_config::generate_config_from_yaml_definition;

fn main() {
    println!("cargo:rerun-if-changed=./foa_sta_config.yml");
    let cfg_yaml = std::fs::read_to_string("./foa_sta_config.yml").unwrap();
    let _ = generate_config_from_yaml_definition(&cfg_yaml, true, true, None).unwrap();
}
