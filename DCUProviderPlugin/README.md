# DCU Provider Plugin für X-Plane 12

X-Plane Plugin zur Kommunikation zwischen dem DCU (Data Communication Unit) und X-Plane über serielle Schnittstelle.

## Build

### macOS

```bash
./build-macos.sh          # Release Build
./build-macos.sh debug    # Debug Build mit Symbolen
```

### Windows (Cross-Compile mit MinGW)

```bash
./build-windows.sh
```

### Alle Plattformen

```bash
./build-all.sh
```

## Debugging

Das Plugin unterstützt vollständiges Source-Level-Debugging mit VS Code und Breakpoints.

### Voraussetzungen

- VS Code mit der **CodeLLDB** Extension (bereits installiert)
- Debug-Build des Plugins (automatisch bei F5)

### Debugging-Schritte

#### 1. Breakpoints setzen

- Öffne eine Source-Datei (z.B. `src/Plugin.cpp` oder `src/DCUProvider.cpp`)
- Klicke links neben die Zeilennummer, um einen Breakpoint zu setzen (roter Punkt)
- Breakpoints können auch während des Debuggings gesetzt/entfernt werden

#### 2. Debug-Session starten

**Option A: X-Plane mit Debugger starten (empfohlen)**

> ⚠️ **Wichtig:** Öffne zuerst eine C++ Datei (z.B. `src/Plugin.cpp`), bevor du F5 drückst, oder wähle die Debug-Konfiguration explizit aus!

- Öffne eine C++ Source-Datei aus dem Projekt
- Drücke **F5** oder
- Gehe zu "Run and Debug" (⌘⇧D), wähle **"Debug X-Plane Plugin"** im Dropdown und klicke auf den grünen Play-Button

Was passiert:
- Das Plugin wird automatisch mit Debug-Symbolen gebaut (`Build Debug` Task)
- X-Plane startet mit angehängtem Debugger
- Wenn dein Plugin geladen wird und ein Breakpoint erreicht wird, stoppt die Ausführung
- Du siehst Variablen, Call Stack, etc. in VS Code

**Option B: An laufendes X-Plane anhängen**

- Starte X-Plane normal
- Drücke **F5** und wähle **"Attach to X-Plane"**
- Nützlich, wenn X-Plane bereits läuft und du nicht neu starten möchtest

#### 3. Debug-Befehle

Während des Debuggings:

- **F10** - Step Over (nächste Zeile ausführen)
- **F11** - Step Into (in Funktionsaufruf springen)
- **⇧F11** - Step Out (aus aktueller Funktion zurück)
- **F5** - Continue (weiterlaufen bis nächster Breakpoint)
- **⇧F5** - Stop Debugging (Debugger beenden)

#### 4. Variablen inspizieren

- Im Debug-View (links) werden alle lokalen Variablen angezeigt
- Hovere mit der Maus über Variablen im Code
- Nutze die "Watch"-Section für spezifische Ausdrücke
- Die "Call Stack"-Ansicht zeigt die Funktionsaufruf-Hierarchie

### Debug-Konfigurationen

Die `.vscode/launch.json` enthält zwei Konfigurationen:

1. **Debug X-Plane Plugin** - Startet X-Plane mit Debugger
2. **Attach to X-Plane** - Verbindet mit laufendem X-Plane

### Logging

Das Plugin verwendet `XPLMDebugString()` für Log-Ausgaben. Diese erscheinen in:

```
/Volumes/1TBSSD/XPlane/X-Plane 12/Log.txt
```

Log-Datei in Echtzeit verfolgen:

```bash
tail -f "/Volumes/1TBSSD/XPlane/X-Plane 12/Log.txt" | grep "DCUProvider"
```

### Troubleshooting

**Breakpoints werden nicht getroffen:**
- Stelle sicher, dass du einen Debug-Build verwendest (`./build-macos.sh debug`)
- Prüfe, dass das Debug-Plugin installiert wurde:
  ```
  /Volumes/1TBSSD/XPlane/X-Plane 12/Resources/plugins/DCUProvider/64/mac.xpl
  ```
- Vergewissere dich, dass das Plugin in X-Plane geladen wird (siehe Log.txt)

**X-Plane startet nicht:**
- Prüfe den Pfad in `.vscode/launch.json`
- Stelle sicher, dass X-Plane nicht bereits läuft

**Symbole nicht gefunden:**
- Build das Plugin erneut mit `./build-macos.sh debug`
- Die CMake-Konfiguration fügt automatisch `-g -O0` im Debug-Modus hinzu

## Projektstruktur

```
DCUProviderPlugin/
├── src/                    # Source-Code
│   ├── Plugin.cpp         # Plugin-Einstiegspunkte
│   ├── DCUProvider.cpp    # Hauptlogik
│   ├── SerialPort.cpp     # Serielle Kommunikation
│   ├── TransportLayer.cpp # Protokoll-Implementierung
│   └── ...
├── SDK/                   # X-Plane SDK
├── build-debug/           # Debug-Build-Ausgabe
├── build-macos/           # Release-Build-Ausgabe
├── .vscode/               # VS Code Konfiguration
│   ├── launch.json       # Debug-Konfigurationen
│   └── tasks.json        # Build-Tasks
└── CMakeLists.txt        # Build-Konfiguration

```

## Installation

Das Build-Skript installiert das Plugin automatisch nach:

```
/Volumes/1TBSSD/XPlane/X-Plane 12/Resources/plugins/DCUProvider/64/mac.xpl
```

Nach der Installation X-Plane neu starten, um das Plugin zu laden.
