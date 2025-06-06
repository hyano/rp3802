# RP3802: Tiny YM3802 emulator by Raspberry Pi Pico

RP3802 is a pin-level emulator of the YAMAHA MIDI control chip YM3802 using a Raspberry Pi Pico.

## Notes

* Please use at your own risk.
* The input/output voltage of the Raspberry Pi Pico's GPIO is 3.3V, while the input/output voltage of the YM3802 is 5V. However, no level shifter is used for cost reduction.
* The Raspberry Pi Pico is overclocked to 250MHz for operation.
* Serial I/F settings are fixed at 31,250bps, 8bits, no parity.
* Supports only major functions for X68000 games and utilities.

## Pin assignment

| Pico          | Direction | Description       | Comment       |
| ------------- | --------- | ----------------- | ------------- |
| GPIO 0        | Output    | YM3802 TxD        | UART TX       |
| GPIO 1        | Input     | YM3802 RxD        | UART RX       |
| GPIO 2-4      | Input     | YM3802 A0-A2      |               |
| GPIO 5-12     | Input     | YM3802 D0-D7      |               |
| GPIO 13       | Input     | /CS \| /RD        | <-- GPIO 20   |
| GPIO 14       | Input     | /CS \| /WR        | <-- GPIO 21   |
| GPIO 15       | Input     | YM3802 /IC        |               |
| GPIO 16       | Outut     | YM3802 /IRQ       |               |
| GPIO 17       | Input     | YM3802 /RD        |               |
| GPIO 18       | Input     | YM3802 /CS        |               |
| GPIO 19       | Input     | YM3802 /WR        |               |
| GPIO 20       | Output    | /CS \| /RD        | --> GPIO 13   |
| GPIO 21       | Output    | /CS \| /RD        | --> GPIO 14   |
| GPIO 28       | Output    | DEBUG             |               |

## Implementation status

| internal block functions  | Status                |
| ------------------------  | --------------------- |
| Transmitter               | Implemented           |
| FIFO-Tx                   | Implemented           |
| Receiver                  | Implemented           |
| FIFO-Rx                   | Implemented           |
| Break detector            | Not implemented       |
| FIFO-ITx                  | Work in progress      |
| Idle detector             | Not implemented       |
| Off-line detector         | Not implemented       |
| MIDI clock filter         | Implemented           |
| Address hunter            | Not implemented       |
| SYNC detector             | Not implemented       |
| MIDI clock timer          | Implemented           |
| SYNC controller           | Not implemented       |
| CLICK counter             | Implemented           |
| MIDI clock interpolator   | Not implemented       |
| Recording counter         | Not implemented       |
| Playback counter          | Not implemented       |
| MIDI message detector     | Work in progress      |
| FIFO-IRx                  | Work in progress      |
| MIDI message distributor  | Work in progress      |
| General-purpose timer     | Implemented           |
| I/O controller            | Not implemented       |
| IRQ controller            | Implemented           |

## License

RP3802 is licensed under the MIT License. See [LICENSE](LICENSE).
