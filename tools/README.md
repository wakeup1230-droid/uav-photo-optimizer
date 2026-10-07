# tools

Third-party tools are installed here locally and are **not** committed (see `.gitignore`).

## ExifTool (required)

Windows:

1. Download the Windows 64-bit package from https://exiftool.org
2. Rename `exiftool(-k).exe` to `exiftool.exe`
3. Place `exiftool.exe` **and** the `exiftool_files` folder in `tools/exiftool/`

```text
tools/
└─ exiftool/
   ├─ exiftool.exe
   └─ exiftool_files/
```

Linux / macOS: install from your package manager (e.g. `apt install libimage-exiftool-perl`).

Lookup order used by `ExifToolAdapter`: explicit path → environment variable
`UAV_EXIFTOOL_PATH` → `tools/exiftool/exiftool(.exe)` → system `PATH`.

License information: see `THIRD_PARTY_NOTICES.md`.
