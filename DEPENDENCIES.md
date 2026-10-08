# Dipendenze

Elenco delle dipendenze necessarie per compilare ed eseguire il progetto.
Versioni "testate" = presenti sulla macchina di sviluppo il 2026-09-29.

## 1. Host — build CMake (`main.cpp`, test, `lqr_design`)

| Dipendenza | Versione minima | Testata | Note |
|---|---|---|---|
| CMake | 3.16 | 3.28.3 | CTest incluso in CMake |
| Compilatore C++17 | — | g++ 13.3.0 | `CMAKE_CXX_STANDARD 17` |
| Eigen3 | 3.3 (con config CMake) | 3.4.0 | `find_package(Eigen3 3.3 REQUIRED NO_MODULE)`; su Ubuntu `libeigen3-dev` |

```bash
sudo apt install cmake g++ libeigen3-dev
cmake -S furuta -B build
cmake --build build
ctest --test-dir build --output-on-failure   # lqr_gains, lqr_control
./build/furuta
./build/lqr_design
```

## 2. Firmware board (`furuta/src/main.ino`) — non compilato da CMake

| Dipendenza | Versione | Note |
|---|---|---|
| Arduino IDE 2.x (o `arduino-cli`) | 2.3.10 | AppImage sul Desktop |
| Core "esp32 by Espressif Systems" | 3.3.11 | board ESP32-S3, FQBN `esp32:esp32:esp32s3` |
| Simple FOC (SimpleFOC) | 2.4.0 | pilotaggio BLDC, `loopFOC`, sensori I2C |
| ArduinoEigen (libreria `Eigen` di hideakitai) | 0.3.2 | header `<EigenDense.h>` usato da `dynamics.hpp` |
| ArxTypeTraits | 0.3.2 | dipendenza di ArduinoEigen (`depends=` in `library.properties`) |
| `furuta` | 0.1.0 | copia locale di `furuta/include` in `~/Arduino/libraries/furuta/src`; **da risincronizzare quando cambia `furuta/include`** |

L'hardware minimo per l'esecuzione è descritto in README §2: ESP32-S3, due AS5600
su bus I2C separati (stesso indirizzo `0x36`), motore BLDC 8 poli con driver
3PWM, alimentazione 24 V, seriale USB (`/dev/ttyACM0`, 115200 baud).

## 3. Logger seriale Python (`furuta/data/data_extraction.py`)

| Dipendenza | Versione | Note |
|---|---|---|
| Python | 3.x | 3.12.3 |
| pyserial | — | 3.5, presente nel venv `.venv/` del repository |

Solo `serial` è di terze parti; `csv`, `datetime`, `os` sono standard library.
`numpy` è presente nel venv ma non usato; `scipy`/`matplotlib` non servono
(il riferimento a SciPy nel README è solo documentale).

```bash
.venv/bin/python furuta/data/data_extraction.py   # richiede /dev/ttyACM0 connesso
```

## 4. Controllo ottimo / MPC offline (opzionali, non usati dal codice attuale)

Nessun sorgente del repository referenzia acados o CasADi: servono solo per
progettazione MPC/ottima lato host, non per il firmware.

| Dipendenza | Versione | Note |
|---|---|---|
| acados | v0.6.0-53 (commit `e35cc32d9`) | sorgente in `~/acados`, build CMake con BLASFEO/HPIPM inclusi (`~/acados/lib`) |
| acados_template | 0.5.1 | interfaccia Python, installata in modalità editable da `~/acados/interfaces/acados_template` |
| CasADi | 3.8.1 | pacchetto Python nel venv `.venv/`; richiesto da `acados_template` (`casadi<=3.8.1`) |

Setup in `~/.bashrc` (da correggere: le virgolette impediscono l'espansione
di `~`, le variabili contengono il percorso letterale `~/acados`):

```bash
export ACADOS_SOURCE_DIR="$HOME/acados"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$HOME/acados/lib"
```

## 5. Sviluppo (opzionali)

- VS Code + CMake Tools + estensione C/C++: usano `build/compile_commands.json`
  (generato da `CMAKE_EXPORT_COMPILE_COMMANDS ON`).
- `git` per il versionamento.
