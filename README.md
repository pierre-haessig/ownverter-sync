# OwnVerter application: islanded inverter with grid synchronization

This repository host embedded microcontroller code for using the [OwnTech OwnVerter](https://www.owntech.io/ownverter/) board as an **islanded three-phase inverter**, that is *not* grid connected, but instead feeding a three-phase load (R, RL...).

However, compared to https://github.com/pierre-haessig/ownverter-islanded, it adds ability to **synchronize to the grid**, with grid voltages measurement + **PLL**. Still, no power is exchanged with the grid.

Remark: the "islanded" qualifier may be a bit misleading. Here it means that the inverter works in the simplest operation mode: **open loop** (no regulation), simply generating a three-phase voltages of given frequency and amplitude.

This code is used in the context of a power electronics course at CentralSupélec, Rennes campus: [http://éole.net/courses/onduleur/](http://éole.net/courses/onduleur/) (in French).

## Experiment schematics

Wiring diagram:

![wiring diagram of the inverter](images/ownverter_wiring_inverter_load-grid-sync.png)

## Usage

This code derives from the [OwnTech Power API Core repository](https://github.com/owntech-foundation/Core). It is designed to be used with VS Code and PlatformIO. The usage of this type of repository is documented at https://docs.owntech.org (e.g. Getting Started section).

