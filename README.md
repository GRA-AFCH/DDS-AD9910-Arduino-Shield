# DDS-AD9910-Arduino-Shield
DDS (Direct Digital Synthesis) Analog Devices AD9910 Arduino Shield by GRA &amp; AFCH

Easy connection to Arduino Mega without additional wires and converters. All functions of the DDS AD9910 are brought to the contacts of the Arduino Mega thanks to this you can fully reveal all the capabilities of the DDS AD9910.

Key Benefits:
*Low harmonics no more than -60dB. An output RF transformer is used for the correct operation of the current mirror.
*Small spur
*4 layer board. Signal lines TOP and Bottom, inner layers Ground and Power.
*Low Noise LDO Stabilizers
*Separate power supply for all analog and digital lines (1.8 and 3.3V), 5pcs IC voltage stabilizers are used. Additionally, there is an RF Ferrite bead interchange.
*High-speed decoupling Level converter and TTL 5V matching
*4 types of generation are possible:
1. XO - Crystal 25Mhz 20ppm internal oscillator with PLL at 1 GHz,
2. TCXO - 10Mhz 1ppm external oscillator PLL at 1 GHz,
3. EGEN - external generator up to 1 GHz
4. OCXO - Oven Controlled Crystal Oscillators deliver the ultimate piezo electric performance with stabilities down to less than ± 1ppb.
(*) additionally used balancing transformer for TCXO, EGEN and OCXO options

*Easy to connect OLED display
*Control buttons for control via the program menu.

# Switching a clock source made by next components:
<pre>
|-------------------------------|-----------------------------------|-----------------|
|      Clock source             |      Capacitors                   |    Resistors    |
|   (only one at a time)        | C20  |  C22  |  C18,C19 | C14,C17 |  XTAL | REF_CLK |
|-------------------------------------------------------------------------------------|
| XO - Crystal Oscillator (Z1)  |  -   |   -   |    V     |    X    |   V   |    X    |
| TCXO - Oscillator 1ppm (Z2)   |  V   |   X   |    X     |    V    |   X   |    V    |
| OCXO - Oscillator 0.1ppm (Z3) |  X   |   V   |    X     |    V    |   X   |    V    |
| EGEN - External Generator     |  X   |   X   |    X     |    V    |   X   |    V    |
|-------------------------------------------------------------------------------------|
</pre>
