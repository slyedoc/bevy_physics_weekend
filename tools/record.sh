
cargo build --release --example=stack --features="timeout"
# copying becuase of assets folder, and it didn't link a symlink
cp ./target/release/examples/stack ./stack

perf record --call-graph=lbr ./stack -e
