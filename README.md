

<img src="https://github.com/CR-3D/SliCR-3D/assets/78646047/92d49a94-9967-4bdf-afae-f8af608e37f8" width="200"> 

[![Discord](https://img.shields.io/discord/856802286588002304?color=7289da&logo=discord&logoColor=white)](https://github.com/supermerill/SliCR-3D/issues/611#issuecomment-907833287) [![Website](https://img.shields.io/website?url=https%3A%2F%2Fwww.cr3d.de&up_message=ONLINE&down_message=OFFLINE&style=flat&label=CR3D&color=%231a7476)](https://www.cr3d.de)

# SliCR-3D

**SliCR-3D** is a fork of SuperSlicer and PrusaSlicer, which are both forks of Slic3r. Previously known as Slic3r++.

Prebuilt Windows, Linux, and macOS 64-bit releases are available through the [git releases page](https://github.com/CR-3D/SliCR-3D/releases). Linux & macOS builds are compiled but not extensively tested, so please report any bugs. For installation, check the badges above. Windows users can use chocolatey or scoop. Nightly builds are available through the [git actions page](https://github.com/CR-3D/SliCR-3D/actions). Click on the build for your platform and then on the archive name (`nightly_win64` or `SliCR-3D-gtk2.AppImage` or `nightly_macos.dmg`) in the "Artifacts" section in the top right corner.

**SliCR-3D** takes 3D models (STL, OBJ, AMF) and converts them into G-code instructions for FFF printers or PNG layers for mSLA 3D printers. It's compatible with any modern printer based on the RepRap toolchain using firmware such as Marlin, Prusa, Klipper, etc.

**SliCR-3D** is based on [PrusaSlicer](https://github.com/prusa3d/PrusaSlicer) by Prusa Research and [SuperSlicer](https://github.com/supermerill/SuperSlicer) by supermerill, which are both based on [Slic3r](https://github.com/Slic3r/Slic3r) by Alessandro Ranellucci and the RepRap community.

For detailed compilation instructions, refer to the [wiki](https://github.com/CR-3D/SliCR-3D/wiki) and the [documentation directory](doc/).

## Key Features

- Custom-made calibration tests
- Fine-tuning options for top surface quality
- "Denser infill" option for solid top layers
- Improved thin walls
- Options for adjusting hole dimensions/geometry
- Better overhangs handling
- Reworked brim with more options
- New seam options
- Built-in calibration prints and object generation script
- Joining perimeters to avoid travel moves
- Numerous other options and improvements
- Includes all features from Slic3rPE/PrusaSlicer
- 3D preview (including multi-material files)
- Customizable G-code macros and output filename with variable placeholders
- Support for post-processing scripts
- Cooling logic controlling fan speed and dynamic print speed


Complete changelog available [here](https://github.com/CR-3D/SliCR-3D/wiki).

## Development

**Language**: Primarily written in C++.

**Compilation**: Instructions for compiling are available for [Linux](doc/How%20to%20build%20-%20Linux%20et%20al.md), [macOS](doc/How%20to%20build%20-%20Mac%20OS.md), and [Windows](doc/How%20to%20build%20-%20Windows.md).

**Contribution**: Contributions are welcome! Please open an issue to discuss proposed changes before submitting a pull request.

## License

SliCR-3D is licensed under the GNU Affero General Public License, version 3. It is based on PrusaSlicer, which is also licensed under the GNU Affero General Public License, version 3. PrusaSlicer, in turn, is based on Slic3r, which is also licensed under the GNU Affero General Public License, version 3.

This license ensures that if you use any part of this software, your software must be released under the same license.
### Directory structure

* `package/`: the scripts used for packaging the executables
* `src/`: the C++ source of the `slic3r` executable and the CMake definition file for compiling it
* `src/slic3r/GUI`: The C++ GUI.
* `tests`: Test suite for Slic3r. Implemented with [Catch2](https://github.com/catchorg/Catch2)
* `utils/`: various useful scripts
* `src/libslic3r/`: C++ sources for libslic3r
### Can I help?

Sure! You can do the following to find things that are available to help with:
* Add an issue to the GitHub tracker **if it isn't already present**.

Before sending patches and pull requests contact me (preferably through opening a GitHub issue or commenting on an existing, related, issue) to discuss your proposed
changes. This way we can ensure that nobody wastes their time and no conflicts arise in development.

## License and attribution

## Licensing and attribution

Slic3r is licensed under the _GNU Affero General Public License, version 3_.

PrusaSlicer is licensed under the _GNU Affero General Public License, version 3_.
PrusaSlicer is owned by Prusa Research.
PrusaSlicer is originally based on Slic3r by Alessandro Ranellucci.

The _GNU Affero General Public License, version 3_ ensures that if you **use** any part of this software in any way (even behind a web server), your software must be released under the same license.

### Acknowledgements

The main author of Slic3r is Alessandro Ranellucci (@alranel, *Sound* in IRC, [@alranel](http://twitter.com/alranel) on Twitter), who started the project in 2011.

Joseph Lenox (@lordofhyphens, *LoH* in IRC, [@LenoxPlay](http://twitter.com/LenoxPlay) on Twitter) is the current co-maintainer.

Contributions by Henrik Brix Andersen, Vojtech Bubnik, Nicolas Dandrimont, Mark Hindess, Petr Ledvina, Y. Sapir, Mike Sheldrake, Kliment Yanev and numerous others. Original manual by Gary Hodgson. Slic3r logo designed by Corey Daniels, <a href="http://www.famfamfam.com/lab/icons/silk/">Silk Icon Set</a> designed by Mark James, stl and gcode file icons designed by Akira Yasuda.

### How can I invoke Slic3r using the command line?

The command line is documented in the relevant [manual page](https://manual.slic3r.org/advanced/command-line).
