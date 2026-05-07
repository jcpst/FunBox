# Dwarf Planet Series

Experimental effects exploring new territory for the Funbox platform.

- [Ceres](Ceres/)     : An envelope filter with synthy waveshape distortion and resonant filter.
- [Makemake](Makemake/) : A pitch-following sample drone with real-time transposition.
- [Orcus](Orcus/)     : A momentary octave pitch shifter with modulation and glide.
- [Quaoar](Quaoar/)   : A three-band semi-parametric equalizer.
- [Sedna](Sedna/)     : Rainbow Machine-inspired polyphonic pitch warping with Magic regeneration.

## Adding a New Dwarf Planet

1. Copy `../Template/` to a new folder here (e.g., `NewName/`)
2. Rename `template.cpp` → `newname.cpp`, update `TARGET` and `CPP_SOURCES` in `Makefile`
3. Update library paths in `Makefile` to use `../../../` instead of `../../` (see existing Makefiles in this directory)
4. Fill in switch handling, audio processing, and parameter ranges
5. Add a `README.md` with the controls table

## Build

```sh
cd software/DwarfPlanets/<Name>
make
make program-dfu
```

