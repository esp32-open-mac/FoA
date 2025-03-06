SSID=$3 DEFMT_LOG=$4 cargo run -r --features $2 --target xtensa-$2-none-elf --bin $1
