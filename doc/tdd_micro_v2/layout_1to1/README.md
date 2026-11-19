# TDD Micro AASD-SERVO MMOS FFB v2.0: 1:1 layout (reconstructed)

Rebuilt on 2026-11-19 from the two CAM renders (top + bottom) and the OpenFFBoard F407VG_DISCO pin map.
This is reverse-engineered and unverified. It is not an official Thanos/tronicgr file.

## Files
| File | What it is |
|---|---|
| `tdd_micro_v2_layout.kicad_pcb` | KiCad 7 format (also opens in 8/9). Board outline, 30 footprints, 222 pads, 682 track segments, 56 vias, GND pours on both layers, and the silkscreen artwork vectorised 1:1 |
| `tdd_micro_v2_layout_1to1_A4.pdf` | Top and bottom at actual size on A4. Print at **100% / Actual size**, then check the 50 mm bar |
| `images/compare_*.png` | Original render (left) next to the recreation (right) |
| `tools/` | Python scripts that did the extraction (`make_pcb.py out`, then `render.py`, then `print_pdf.py x.pdf`) |

## Geometry (measured)
- Scale: 29.345 px/mm. Header pitch fits 2.54 mm (σ 0.04 px), DB25 fits 2.77 × 2.84 mm.
- Board: **79.35 × 80.95 mm**, square corners.
- Every hole matches its footprint pad within **0.10 mm**. Most are within 0.03 mm.
- Holes: headers 0.8/1.8 mm, DB9 0.8/1.75, DB25 1.05/1.65, LED 0.85/1.5, E-STOP 1.1 with 4.0×2.0 oval pads.
  Mounting holes: corners 3.5/6.0, DB9 shells 3.0/3.95, DB25 shell 3.15/3.75. Vias 0.3/0.7.
- Tracks are mostly 0.254 mm, with some at 0.4–0.8 mm. GND pour clearance is about 0.3 mm.

## After opening in KiCad
1. Press **B** to fill the GND zones. They are saved unfilled.
2. Run DRC and look at the ratsnest. Traces were vectorised automatically. Where silkscreen covered a trace (for example the big "THANOS" letters and the module header labels on the bottom), it may be broken or missing. Ratsnest lines show those gaps. Tracks without a net are fragments whose ends could not be resolved.
3. The DIY-MORE module is one footprint, **J1**, with three female headers. Pads are named `L1..L36` (left 2×18), `R1..R32` (right 2×16) and `B1..B24` (bottom 2×12). Each pad's *pin function* is its MCU pin (PE9, PA0, …).

The pad numbers do **not** match the symbols in `../tdd_micro_v2.kicad_sch`. Do not run "Update PCB from schematic".

## Connections confirmed from the copper
PE11→DB25-7 (DIR) · 5V→DB25-9 · PA2→DB25-15 (index) · PA1→DB25-20 · PB1→pedal GAS · PB0→BRAKE · PC5→Z header ·
PA4→shifter CS · PE0/PC10/PC11/PC7→button header · PD12→LED1 (green) anode · PD15→LED2 (blue) anode ·
DB25-25 (Vref)→torque-select jumper (PA4 pad) · U1 VCC↔C1 · U1 VOUTB↔left "B" jumper · U1 INA/INB↔right "A"/"B" jumpers ·
DB25 shell → "Shield Ground" jumper (bottom side) → GND.

## Differences from the earlier schematic (`../tdd_micro_v2.kicad_sch`)
- **E-STOP terminal**: pin 1 goes to **DB25-6 (SigIn1 / SON)** and pin 2 to GND. So the E-STOP switch enables the servo when closed. The schematic had "R4, EN source unknown". There is no R4 on the board.
- **AREF** on the SWD header is tied to **3V3**.
- **Encoder on DB25**: PA1 (ENC_B) is routed to **pin 20**, and the bottom silkscreen labels **pin 18 "ENC A"**. The schematic has these swapped.
- **DB25-1** looks unconnected. The schematic says GND.
- **U1 (LTC2644) pins 7–12**, from the chip silkscreen: 7 PD, 8 IN_B, 9 IN_A, 10 REF, 11 REFSEL, 12 GND. The schematic numbered these in the opposite order.
- The **solder jumpers are 2-pad**, not 3-pad. There are seven on top plus "Shield Ground" on the bottom:
  - right A/B connect DIR/PWM to DAC IN_A/IN_B;
  - left A/B pick VOUTA or VOUTB;
  - PA5 / PA4 / DAC pick what drives DB25-25 (Vref).

  The left-jumper and PA5/DAC connections are inferred from the layout, not proven by the copper.
- **LEDs** run MCU pin → LED anode → LED cathode → 470 Ω → GND, so the resistor is on the low side. Each LED footprint has both 3 mm THT pads and 1206 SMD pads.
- **"SPI2 pads (J9)"** are not a separate part. They are the module's bottom-header pins, labelled on the bottom silkscreen.
- The two **BOOT** pins on the left header are tied together.

## Still open
- **U1 IDLSEL (pin 4)**: the chip's silkscreen outline lies exactly on the pad ends, so I couldn't tell whether it ties to the GND pour. Measure it on the real board.
- The C1/C2/C3 GND sides, REFSEL and DB9 shell grounding come from the schematic, not the copper.
