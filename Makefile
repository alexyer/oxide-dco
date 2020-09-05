debug:
	cargo build
release:
	cargo build --release
run: debug
	cargo run
run-release: release
	qemu-system-arm -cpu cortex-m3 -machine stm32-f103c8 -nographic -semihosting -gdb tcp::3333 -S -kernel target/thumbv6m-none-eabi/release/oxide-dco
gdb:
	gdb -q target/thumbv6m-none-eabi/debug/oxide-dco --command openocd.gdb
gdb-release:
	gdb -q target/thumbv6m-none-eabi/release/oxide-dco --command openocd.gdb
openocd:
	openocd --file openocd.cfg