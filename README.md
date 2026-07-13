<img src="https://github.com/CR-3D/SliCR-3D/assets/78646047/92d49a94-9967-4bdf-afae-f8af608e37f8" width="200">

[![Discord](https://img.shields.io/discord/856802286588002304?color=7289da&logo=discord&logoColor=white)](https://github.com/supermerill/SliCR-3D/issues/611#issuecomment-907833287) [![Website](https://img.shields.io/website?url=https%3A%2F%2Fwww.cr3d.de&up_message=ONLINE&down_message=OFFLINE&style=flat&label=CR3D&color=%231a7476)](https://www.cr3d.de)

# SliCR-3D

SliCR-3D ist ein Fork von SuperSlicer und PrusaSlicer, die wiederum beide auf Slic3r basieren. Das Projekt war früher unter dem Namen Slic3r++ bekannt.

Vorkompilierte 64-Bit-Versionen für Windows, Linux und macOS stehen auf der [Release-Seite](https://github.com/CR-3D/SliCR-3D/releases) bereit. Die Builds für Linux und macOS werden erstellt, aber nicht umfassend getestet – bitte melden Sie Fehler. Installationshinweise finden Sie über die Badges oben. Windows-Nutzer können auch Chocolatey oder Scoop verwenden. Nightly Builds sind über die [GitHub-Actions-Seite](https://github.com/CR-3D/SliCR-3D/actions) verfügbar. Wählen Sie den Build für Ihre Plattform und laden Sie im Bereich „Artifacts“ das passende Archiv herunter (`nightly_win64`, `SliCR-3D-gtk2.AppImage` oder `nightly_macos.dmg`).

SliCR-3D wandelt 3D-Modelle (STL, OBJ, AMF) in G-Code-Anweisungen für FFF-Drucker oder PNG-Schichten für mSLA-3D-Drucker um. Es ist mit modernen Druckern auf Basis des RepRap-Ökosystems kompatibel, etwa mit Marlin-, Prusa- oder Klipper-Firmware.

SliCR-3D basiert auf [PrusaSlicer](https://github.com/prusa3d/PrusaSlicer) von Prusa Research und [SuperSlicer](https://github.com/supermerill/SuperSlicer) von supermerill. Beide Projekte basieren auf [Slic3r](https://github.com/Slic3r/Slic3r) von Alessandro Ranellucci und der RepRap-Community.

## Systemanforderungen

Die Systemanforderungen hängen davon ab, ob Sie SliCR-3D nur zur Druckvorbereitung verwenden oder auch komplexe 3D-Modelle erstellen.

### Druckvorbereitung und Slicing

- **CPU:** Ein Prozessor mit mehreren Kernen und Threads wird empfohlen, da viele Berechnungen beim Slicing parallel ausgeführt werden.
- **Arbeitsspeicher:** Mindestens 4 GB RAM; 8 GB oder mehr werden empfohlen.
- **Grafik:** Integrierte Grafik, beispielsweise Intel HD Graphics, ist in der Regel ausreichend. SliCR-3D benötigt OpenGL 3.2 oder neuer.

### Unterstützte Betriebssysteme

- **Windows:** 64-Bit Windows 7 oder neuer. SliCR-3D-Releases sind ausschließlich 64-Bit.
- **macOS:** macOS 10.11 (El Capitan) oder neuer. Für Versionen, die auf PrusaSlicer neuer als 2.4.2 basieren, ist macOS 10.12 (Sierra) oder neuer erforderlich.
- **Linux:** Die meisten gängigen Linux-Distributionen.
- **Chromebook:** Aktuelles Chrome OS mit Debian 10.7 oder neuer.

### 3D-Modellierung und CAD

Für das Entwerfen komplexer Bauteile zusätzlich zum Slicing wird leistungsfähigere Hardware empfohlen, beispielsweise ein Intel Core i7 oder AMD Ryzen 7 sowie mindestens 16 GB RAM.

Ausführliche Anleitungen zum Kompilieren finden Sie im [Wiki](https://github.com/CR-3D/SliCR-3D/wiki) sowie in der Dokumentation für [Linux](doc/How%20to%20build%20-%20Linux%20et%20al.md), [macOS](doc/How%20to%20build%20-%20Mac%20OS.md) und [Windows](doc/How%20to%20build%20-%20Windows.md).

## Hauptfunktionen

- Speziell entwickelte Kalibrierungstests
- Feinabstimmung der Qualität von Deckschichten
- Option „Dichteres Infill“ für massive Deckschichten
- Verbesserte dünne Wände
- Einstellungen zur Anpassung von Lochmaßen und Geometrie
- Verbesserte Behandlung von Überhängen
- Überarbeiteter Brim mit zusätzlichen Optionen
- Neue Optionen für die Naht
- Integrierte Kalibrierungsdrucke und Skript zum Erzeugen von Objekten
- Verbundene Perimeter zur Vermeidung von Fahrbewegungen
- Viele weitere Optionen und Verbesserungen
- Alle Funktionen aus Slic3rPE/PrusaSlicer
- 3D-Vorschau, einschließlich Mehrmaterial-Dateien
- Anpassbare G-Code-Makros und Ausgabedateinamen mit Platzhaltern
- Unterstützung für Nachbearbeitungsskripte
- Kühllogik zur Steuerung der Lüfterdrehzahl und dynamischen Druckgeschwindigkeit

Das vollständige Changelog ist im [Wiki](https://github.com/CR-3D/SliCR-3D/wiki) verfügbar.

## Entwicklung

- **Sprache:** Überwiegend in C++ geschrieben.
- **Kompilieren:** Anleitungen stehen für [Linux](doc/How%20to%20build%20-%20Linux%20et%20al.md), [macOS](doc/How%20to%20build%20-%20Mac%20OS.md) und [Windows](doc/How%20to%20build%20-%20Windows.md) bereit.
- **Beiträge:** Beiträge sind willkommen. Bitte eröffnen Sie vor einem Pull Request ein Issue, um geplante Änderungen zu besprechen.

### Verzeichnisstruktur

- `package/`: Skripte zum Paketieren der ausführbaren Dateien
- `src/`: C++-Quellcode der ausführbaren Datei `slic3r` und CMake-Definitionen zum Kompilieren
- `src/slic3r/GUI`: C++-Benutzeroberfläche
- `src/libslic3r/`: C++-Quellcode von libslic3r
- `tests/`: Test-Suite für Slic3r, implementiert mit [Catch2](https://github.com/catchorg/Catch2)
- `utils/`: Verschiedene hilfreiche Skripte

### Mithelfen

Sie können helfen, indem Sie ein Issue im GitHub-Tracker eröffnen, sofern es noch nicht existiert. Bitte besprechen Sie Änderungen vor dem Einreichen von Patches oder Pull Requests in einem Issue oder einem passenden bestehenden Kommentar. So lassen sich Doppelarbeit und Konflikte vermeiden.

## Lizenz und Danksagungen

SliCR-3D steht unter der GNU Affero General Public License, Version 3 (AGPL-3.0). Auch PrusaSlicer und Slic3r stehen unter dieser Lizenz; PrusaSlicer gehört Prusa Research. Wenn Sie Teile dieser Software verwenden – auch hinter einem Webserver –, muss Ihre Software unter derselben Lizenz veröffentlicht werden.

Slic3r wurde 2011 von Alessandro Ranellucci (@alranel, *Sound* auf IRC, [@alranel](http://twitter.com/alranel) auf Twitter) gestartet. Joseph Lenox (@lordofhyphens, *LoH* auf IRC, [@LenoxPlay](http://twitter.com/LenoxPlay) auf Twitter) ist der aktuelle Co-Maintainer.

Beiträge stammen unter anderem von Henrik Brix Andersen, Vojtech Bubnik, Nicolas Dandrimont, Mark Hindess, Petr Ledvina, Y. Sapir, Mike Sheldrake, Kliment Yanev und vielen weiteren. Das ursprüngliche Handbuch stammt von Gary Hodgson. Das Slic3r-Logo wurde von Corey Daniels gestaltet; das [Silk Icon Set](http://www.famfamfam.com/lab/icons/silk/) von Mark James sowie die STL- und G-Code-Dateisymbole von Akira Yasuda.

## Kommandozeile

Die Kommandozeile ist im entsprechenden [Handbuch](https://manual.slic3r.org/advanced/command-line) dokumentiert.

<details>
<summary>English</summary>

## English documentation

SliCR-3D is a fork of SuperSlicer and PrusaSlicer, which are both forks of Slic3r. Previously known as Slic3r++.

Prebuilt Windows, Linux, and macOS 64-bit releases are available through the [git releases page](https://github.com/CR-3D/SliCR-3D/releases). Linux and macOS builds are compiled but not extensively tested, so please report any bugs. For installation, check the badges above. Windows users can use Chocolatey or Scoop. Nightly builds are available through the [GitHub Actions page](https://github.com/CR-3D/SliCR-3D/actions). Click on the build for your platform and then on the archive name (`nightly_win64`, `SliCR-3D-gtk2.AppImage`, or `nightly_macos.dmg`) in the "Artifacts" section.

SliCR-3D takes 3D models (STL, OBJ, AMF) and converts them into G-code instructions for FFF printers or PNG layers for mSLA 3D printers. It is compatible with modern printers based on the RepRap toolchain using firmware such as Marlin, Prusa, or Klipper.

SliCR-3D is based on [PrusaSlicer](https://github.com/prusa3d/PrusaSlicer) by Prusa Research and [SuperSlicer](https://github.com/supermerill/SuperSlicer) by supermerill, which are both based on [Slic3r](https://github.com/Slic3r/Slic3r) by Alessandro Ranellucci and the RepRap community.

### System requirements

System requirements depend on whether you use SliCR-3D only to prepare prints or also create complex 3D models.

#### Print preparation and slicing

- **CPU:** A multi-core, multi-threaded processor is recommended, as many slicing calculations run in parallel.
- **Memory:** 4 GB RAM minimum; 8 GB or more recommended.
- **Graphics:** Integrated graphics, such as Intel HD Graphics, are generally sufficient. SliCR-3D requires OpenGL 3.2 or later.

#### Supported operating systems

- **Windows:** 64-bit Windows 7 or later. SliCR-3D releases are 64-bit only.
- **macOS:** macOS 10.11 (El Capitan) or later. macOS 10.12 (Sierra) or later is required for versions based on PrusaSlicer newer than 2.4.2.
- **Linux:** Most standard Linux distributions.
- **Chromebook:** Current Chrome OS with Debian 10.7 or later.

#### 3D modelling and CAD

For designing complex parts in addition to slicing, use more powerful hardware, such as an Intel Core i7 or AMD Ryzen 7 processor, with 16 GB RAM or more recommended.

Detailed build instructions are available in the [wiki](https://github.com/CR-3D/SliCR-3D/wiki) and in the documentation for [Linux](doc/How%20to%20build%20-%20Linux%20et%20al.md), [macOS](doc/How%20to%20build%20-%20Mac%20OS.md), and [Windows](doc/How%20to%20build%20-%20Windows.md).

### Key features

- Custom-made calibration tests
- Fine-tuning options for top-surface quality
- "Denser infill" option for solid top layers
- Improved thin walls
- Options for adjusting hole dimensions and geometry
- Improved overhang handling
- Reworked brim with more options
- New seam options
- Built-in calibration prints and an object-generation script
- Joined perimeters to avoid travel moves
- Numerous other options and improvements
- All features from Slic3rPE/PrusaSlicer
- 3D preview, including multi-material files
- Customizable G-code macros and output filenames with variable placeholders
- Support for post-processing scripts
- Cooling logic controlling fan speed and dynamic print speed

The complete changelog is available in the [wiki](https://github.com/CR-3D/SliCR-3D/wiki).

### Development

- **Language:** Primarily written in C++.
- **Compilation:** Instructions are available for [Linux](doc/How%20to%20build%20-%20Linux%20et%20al.md), [macOS](doc/How%20to%20build%20-%20Mac%20OS.md), and [Windows](doc/How%20to%20build%20-%20Windows.md).
- **Contributions:** Contributions are welcome. Please open an issue to discuss proposed changes before submitting a pull request.

#### Directory structure

- `package/`: Scripts used for packaging the executables
- `src/`: C++ source of the `slic3r` executable and CMake definitions for compiling it
- `src/slic3r/GUI`: C++ GUI
- `src/libslic3r/`: C++ sources for libslic3r
- `tests/`: Test suite for Slic3r, implemented with [Catch2](https://github.com/catchorg/Catch2)
- `utils/`: Various useful scripts

#### How can I help?

You can help by opening an issue in the GitHub tracker if it does not already exist. Before sending patches and pull requests, please discuss changes in an issue or a relevant existing comment. This helps avoid duplicated work and conflicts.

### License and acknowledgements

SliCR-3D is licensed under the GNU Affero General Public License, version 3 (AGPL-3.0). PrusaSlicer and Slic3r are licensed under the same terms; PrusaSlicer is owned by Prusa Research. If you use any part of this software, including behind a web server, your software must be released under the same license.

Slic3r was started in 2011 by Alessandro Ranellucci (@alranel, *Sound* in IRC, [@alranel](http://twitter.com/alranel) on Twitter). Joseph Lenox (@lordofhyphens, *LoH* in IRC, [@LenoxPlay](http://twitter.com/LenoxPlay) on Twitter) is the current co-maintainer.

Contributions by Henrik Brix Andersen, Vojtech Bubnik, Nicolas Dandrimont, Mark Hindess, Petr Ledvina, Y. Sapir, Mike Sheldrake, Kliment Yanev, and numerous others. The original manual was written by Gary Hodgson. The Slic3r logo was designed by Corey Daniels, the [Silk Icon Set](http://www.famfamfam.com/lab/icons/silk/) by Mark James, and the STL and G-code file icons by Akira Yasuda.

### Command line

The command line is documented in the relevant [manual page](https://manual.slic3r.org/advanced/command-line).

</details>
